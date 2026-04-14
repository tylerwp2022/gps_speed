#!/usr/bin/env python3
"""
gps_noise_characterizer.py — GPS Noise Diagnostic for gps_speed / imu_compass Tuning
======================================================================================

PURPOSE:
--------
Characterizes GPS noise at standstill by using the hardware status_speed topic as
ground truth for "is the robot actually moving?". Computes the full distribution of
GPS inter-fix distances and implied speeds while stationary, then recommends values
for:

  • min_distance_m               (gps_speed node)
  • min_calibration_speed_m_s    (imu_compass node)
  • max_speed_m_s                (gps_speed sanity cap, if needed)

Also analyzes how often the current imu_compass calibration gate would falsely fire
at standstill, and whether status_speed (odometry) should replace gps_speed as the
calibration gate signal entirely.

USAGE:
------
  # Phase 1: Standstill collection (60+ seconds recommended)
  python3 gps_noise_characterizer.py --robot warthog1 --duration 90 --phase standstill

  # Phase 2: Moving collection (drive at various speeds + directions)
  python3 gps_noise_characterizer.py --robot warthog1 --duration 120 --phase moving

  # Unified: records everything and auto-classifies by status_speed
  python3 gps_noise_characterizer.py --robot warthog1 --duration 180 --phase both

  # Override GPS topic suffix (default: sensors/ublox/fix)
  python3 gps_noise_characterizer.py --robot warthog1 --gps-suffix sensors/geofog/gps/fix

  # Override the standstill speed threshold (default: 0.05 m/s)
  python3 gps_noise_characterizer.py --robot warthog1 --standstill-thresh 0.10

CONFIGURATION:
--------------
Edit the CONFIGURATION section below if your status_speed topic has a different
name or message type. The script defaults to std_msgs/Float64 (a scalar speed in
m/s). If your topic is geometry_msgs/TwistStamped, see STATUS_SPEED_FIELD below.

TOPICS SUBSCRIBED:
------------------
  /{robot_name}/{gps_topic_suffix}   — NavSatFix (raw GPS fixes)
  /{robot_name}/sensors/gps_speed    — Float64   (gps_speed_node output)
  /{robot_name}/status_speed         — Float64   (hardware odometry speed, ground truth)
  /{robot_name}/{imu_topic_suffix}   — Imu       (angular velocity for cal gate analysis)

OUTPUT:
-------
  • Distribution statistics (min/max/mean/std/percentiles) for standstill GPS noise
  • Clear threshold recommendations with margin guidance
  • Analysis of how often current parameters would cause false calibration triggers
  • Suggested imu_compass parameter change: subscribe to status_speed instead of gps_speed
  • CSV dump of all collected samples (optional, use --csv)
"""

#==============================================================================
# CONFIGURATION — edit these if your robot's topics differ
#==============================================================================

# GPS topic suffix (path after /{robot_name}/)
DEFAULT_GPS_SUFFIX = "sensors/ublox/fix"

# IMU topic suffix (path after /{robot_name}/)
DEFAULT_IMU_SUFFIX = "sensors/microstrain/ekf/imu/data"

# status_speed topic name (just the last segment, under /{robot_name}/)
STATUS_SPEED_TOPIC_SUFFIX = "status_speed"

# If status_speed is NOT std_msgs/Float64 but instead geometry_msgs/TwistStamped,
# change this to: lambda msg: abs(msg.twist.linear.x)
# For nav_msgs/Odometry: lambda msg: abs(msg.twist.twist.linear.x)
STATUS_SPEED_FIELD = lambda msg: abs(msg.data)   # noqa: E731

# A robot is "at standstill" when status_speed is below this (m/s)
DEFAULT_STANDSTILL_THRESHOLD_M_S = 0.05

# Earth radius for Haversine
EARTH_RADIUS_M = 6_371_000.0

#==============================================================================
# IMPORTS
#==============================================================================

import argparse
import csv
import math
import signal
import statistics
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg import Float64

#==============================================================================
# DATA MODEL
#==============================================================================

@dataclass
class GpsSample:
    """One raw GPS fix with associated sensor readings at that moment."""
    timestamp_s: float
    lat_deg: float
    lon_deg: float
    fix_status: int           # sensor_msgs/NavSatStatus.status
    gps_speed_m_s: float      # gps_speed_node output at this moment
    status_speed_m_s: float   # hardware odometry at this moment
    yaw_rate_rad_s: float     # IMU angular velocity z at this moment
    # Computed after collection (requires previous sample)
    distance_m: float = 0.0
    dt_s: float = 0.0
    implied_speed_m_s: float = 0.0
    is_standstill: bool = False

#==============================================================================
# COLLECTOR NODE
#==============================================================================

class GpsNoiseCollector(Node):
    """
    ROS2 node that subscribes to GPS, gps_speed, status_speed, and IMU,
    and accumulates GpsSample records for post-hoc analysis.
    """

    def __init__(self, robot_name: str, gps_suffix: str, imu_suffix: str,
                 standstill_threshold: float, duration_s: float, phase: str,
                 csv_path: Optional[str]):
        super().__init__("gps_noise_characterizer")

        self.robot_name = robot_name
        self.standstill_threshold = standstill_threshold
        self.duration_s = duration_s
        self.phase = phase
        self.csv_path = csv_path
        self.start_time = time.time()

        # Latched latest values from non-GPS callbacks
        self.latest_gps_speed = 0.0
        self.latest_status_speed = 0.0
        self.latest_yaw_rate = 0.0

        # Accumulated samples
        self.samples: List[GpsSample] = []
        self.prev_fix: Optional[GpsSample] = None

        # Statistics accumulators (counts)
        self.total_fixes = 0
        self.bad_fixes = 0

        # Topic paths
        gps_topic    = f"/{robot_name}/{gps_suffix}"
        imu_topic    = f"/{robot_name}/{imu_suffix}"
        speed_topic  = f"/{robot_name}/sensors/gps_speed"
        status_topic = f"/{robot_name}/{STATUS_SPEED_TOPIC_SUFFIX}"

        # QoS — sensor data (best-effort) to match drivers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(NavSatFix, gps_topic,
            self._gps_cb, sensor_qos)
        self.create_subscription(Imu, imu_topic,
            self._imu_cb, sensor_qos)
        self.create_subscription(Float64, speed_topic,
            self._gps_speed_cb, sensor_qos)

        # status_speed — try as Float64 (best-effort first, then reliable)
        self.create_subscription(Float64, status_topic,
            self._status_speed_cb, sensor_qos)

        print(f"\n{'='*70}")
        print(f"GPS Noise Characterizer — {robot_name}")
        print(f"{'='*70}")
        print(f"  GPS topic:    {gps_topic}")
        print(f"  IMU topic:    {imu_topic}")
        print(f"  GPS speed:    {speed_topic}")
        print(f"  Status speed: {status_topic}")
        print(f"  Standstill threshold: {standstill_threshold:.3f} m/s")
        print(f"  Collection duration:  {duration_s:.0f}s")
        print(f"  Phase: {phase.upper()}")
        print(f"{'='*70}")
        print()

        if phase == "standstill":
            print("  ACTION: Keep robot completely stationary for the entire run.")
        elif phase == "moving":
            print("  ACTION: Drive the robot at normal operating speeds.")
            print("          Include straight sections (low yaw rate).")
        else:
            print("  ACTION: Collect standstill data, then drive around.")
            print("          The script auto-classifies by status_speed.")
        print()

    # -------------------------------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        self.latest_yaw_rate = msg.angular_velocity.z

    def _gps_speed_cb(self, msg: Float64):
        self.latest_gps_speed = msg.data

    def _status_speed_cb(self, msg: Float64):
        # Uses STATUS_SPEED_FIELD lambda from configuration section.
        # If your topic is a different type, update that lambda.
        try:
            self.latest_status_speed = STATUS_SPEED_FIELD(msg)
        except AttributeError:
            # Wrong message type — warn once
            self.get_logger().warn(
                "status_speed callback received unexpected message type. "
                "Check STATUS_SPEED_FIELD in the CONFIGURATION section.",
                once=True
            )
            self.latest_status_speed = 0.0

    def _gps_cb(self, msg: NavSatFix):
        """Record each GPS fix as a GpsSample."""
        elapsed = time.time() - self.start_time
        if elapsed > self.duration_s:
            return  # Collection window closed

        self.total_fixes += 1

        # Record sample regardless of fix quality — we track bad fixes separately
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            self.bad_fixes += 1
            return  # Don't anchor on bad fixes

        sample = GpsSample(
            timestamp_s=elapsed,
            lat_deg=msg.latitude,
            lon_deg=msg.longitude,
            fix_status=msg.status.status,
            gps_speed_m_s=self.latest_gps_speed,
            status_speed_m_s=self.latest_status_speed,
            yaw_rate_rad_s=self.latest_yaw_rate,
        )

        # Classify standstill using hardware odometry
        sample.is_standstill = (sample.status_speed_m_s < self.standstill_threshold)

        # Compute inter-fix metrics if we have a previous fix
        if self.prev_fix is not None:
            dt = sample.timestamp_s - self.prev_fix.timestamp_s
            dist = haversine_m(
                self.prev_fix.lat_deg, self.prev_fix.lon_deg,
                sample.lat_deg, sample.lon_deg
            )
            sample.distance_m = dist
            sample.dt_s = dt
            sample.implied_speed_m_s = (dist / dt) if dt > 0.001 else 0.0

        self.samples.append(sample)
        self.prev_fix = sample

        # Progress indicator
        if self.total_fixes % 20 == 0:
            standstill_count = sum(1 for s in self.samples if s.is_standstill)
            moving_count = len(self.samples) - standstill_count
            print(f"  [{elapsed:5.1f}s] Fixes: {len(self.samples)}  "
                  f"Standstill: {standstill_count}  Moving: {moving_count}  "
                  f"status_speed: {self.latest_status_speed:.3f} m/s  "
                  f"gps_speed: {self.latest_gps_speed:.3f} m/s",
                  flush=True)

    def is_done(self) -> bool:
        return (time.time() - self.start_time) >= self.duration_s

#==============================================================================
# MATH HELPERS
#==============================================================================

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine distance in meters between two WGS-84 points."""
    lat1, lat2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def percentile(data: List[float], p: float) -> float:
    """Compute the p-th percentile (0–100) of a sorted or unsorted list."""
    if not data:
        return 0.0
    sorted_data = sorted(data)
    k = (len(sorted_data) - 1) * p / 100.0
    lo, hi = int(k), min(int(k) + 1, len(sorted_data) - 1)
    frac = k - lo
    return sorted_data[lo] * (1 - frac) + sorted_data[hi] * frac

def fmt(val: float, decimals: int = 4) -> str:
    return f"{val:.{decimals}f}"

#==============================================================================
# ANALYSIS + REPORTING
#==============================================================================

def analyze(samples: List[GpsSample], standstill_threshold: float,
            current_params: dict, csv_path: Optional[str]):
    """
    Post-process collected samples and print recommendations.
    """

    # Filter out the first sample (no inter-fix metrics yet) and samples
    # with dt=0 (shouldn't happen but be safe)
    valid = [s for s in samples if s.dt_s > 0.001]

    standstill = [s for s in valid if s.is_standstill]
    moving     = [s for s in valid if not s.is_standstill]

    print()
    print("="*70)
    print("ANALYSIS RESULTS")
    print("="*70)
    print(f"  Total GPS fixes recorded:  {len(samples) + 1}")  # +1 for anchor
    print(f"  Valid inter-fix samples:   {len(valid)}")
    print(f"  Classified as STANDSTILL:  {len(standstill)}  "
          f"(status_speed < {standstill_threshold:.3f} m/s)")
    print(f"  Classified as MOVING:      {len(moving)}")

    if len(standstill) < 10:
        print()
        print("  WARNING: Very few standstill samples. Collect at least 60s of")
        print("           stationary data for reliable noise characterization.")
        if len(standstill) == 0:
            print("  ERROR: No standstill data. Check status_speed topic is publishing.")
            print("         Run with --phase standstill and keep the robot still.")
            return

    # -------------------------------------------------------------------------
    # STANDSTILL GPS NOISE CHARACTERIZATION
    # -------------------------------------------------------------------------
    print()
    print("-"*70)
    print("STANDSTILL GPS NOISE (ground truth: robot not moving)")
    print("-"*70)

    # Separate "has previous sample also at standstill" vs mixed transitions
    # For purest noise, we want both samples in the pair to be at standstill
    pure_standstill = [s for s in standstill
                       if s.distance_m >= 0]  # All valid

    dist_vals   = [s.distance_m for s in pure_standstill]
    ispeed_vals = [s.implied_speed_m_s for s in pure_standstill]

    _print_distribution("GPS inter-fix distance (m)", dist_vals)
    _print_distribution("Implied GPS speed (m/s) = dist/dt", ispeed_vals)

    # GPS scatter — how far do fixes wander from the centroid?
    if len(pure_standstill) >= 5:
        mean_lat = statistics.mean(s.lat_deg for s in pure_standstill)
        mean_lon = statistics.mean(s.lon_deg for s in pure_standstill)
        scatter_m = [haversine_m(s.lat_deg, s.lon_deg, mean_lat, mean_lon)
                     for s in pure_standstill]
        _print_distribution("GPS position scatter from centroid (m)", scatter_m)

    # -------------------------------------------------------------------------
    # PARAMETER RECOMMENDATIONS
    # -------------------------------------------------------------------------
    print()
    print("-"*70)
    print("PARAMETER RECOMMENDATIONS")
    print("-"*70)

    p99_dist   = percentile(dist_vals, 99)
    p999_dist  = percentile(dist_vals, 99.9)
    p99_ispeed = percentile(ispeed_vals, 99)
    p999_ispeed = percentile(ispeed_vals, 99.9)

    # Add a safety margin on top of the 99th percentile
    MARGIN = 1.3  # 30% above 99th percentile

    rec_min_distance_m = p999_dist * MARGIN
    rec_min_cal_speed  = p999_ispeed * MARGIN

    print()
    print("  [gps_speed node] min_distance_m:")
    print(f"    Current value:    {current_params.get('min_distance_m', 0.05):.4f} m")
    print(f"    99th percentile:  {p99_dist:.4f} m  (standstill noise)")
    print(f"    99.9th pctile:    {p999_dist:.4f} m")
    print(f"    RECOMMENDED:      {rec_min_distance_m:.4f} m  (99.9th × {MARGIN})")
    print()
    print("    Interpretation: Set min_distance_m above this value and gps_speed")
    print("    will output 0.0 for > 99.9% of standstill GPS jitter.")
    print()

    print("  [imu_compass node] min_calibration_speed_m_s:")
    print(f"    Current value:    {current_params.get('min_calibration_speed_m_s', 1.0):.4f} m/s")
    print(f"    99th pctile implied speed (standstill): {p99_ispeed:.4f} m/s")
    print(f"    99.9th pctile:    {p999_ispeed:.4f} m/s")
    print(f"    RECOMMENDED:      {rec_min_cal_speed:.4f} m/s  (99.9th × {MARGIN})")
    print()
    print("    Interpretation: This is the minimum gps_speed to allow a calibration")
    print("    update. Must be well above the standstill noise floor.")
    print()

    # -------------------------------------------------------------------------
    # FALSE CALIBRATION TRIGGER ANALYSIS
    # -------------------------------------------------------------------------
    print("-"*70)
    print("FALSE CALIBRATION TRIGGER ANALYSIS")
    print("-"*70)
    print()
    print("  How often would the imu_compass calibration gate fire falsely at standstill")
    print("  under various min_calibration_speed_m_s thresholds?")
    print()

    thresholds = [0.1, 0.2, 0.3, 0.5, 0.75, 1.0,
                  round(rec_min_cal_speed, 2), 1.5, 2.0]
    thresholds = sorted(set(thresholds))

    print(f"  {'Threshold (m/s)':<18} {'False triggers':<16} {'False rate':<14} {'Verdict'}")
    print(f"  {'-'*18} {'-'*16} {'-'*14} {'-'*20}")

    total_standstill = len(standstill)
    for thresh in thresholds:
        # False trigger = gps_speed > thresh AND robot is at standstill
        false_triggers = sum(1 for s in standstill
                             if s.gps_speed_m_s > thresh)
        rate = false_triggers / total_standstill if total_standstill > 0 else 0.0
        verdict = ("✓ Recommended" if abs(thresh - rec_min_cal_speed) < 0.05 else
                   "✗ Too low" if rate > 0.01 else
                   "✓ OK" if rate == 0 else
                   "⚠ Some risk")
        marker = " ◄" if abs(thresh - rec_min_cal_speed) < 0.05 else ""
        print(f"  {thresh:<18.2f} {false_triggers:<16} {rate:<14.1%} {verdict}{marker}")

    # -------------------------------------------------------------------------
    # MOVING DATA (if available)
    # -------------------------------------------------------------------------
    if len(moving) >= 10:
        print()
        print("-"*70)
        print("MOVING DATA — GPS speed distribution while robot is driving")
        print("-"*70)
        moving_speeds = [s.gps_speed_m_s for s in moving]
        moving_ispeed = [s.implied_speed_m_s for s in moving]
        _print_distribution("gps_speed output while moving (m/s)", moving_speeds)
        _print_distribution("Implied GPS speed while moving (m/s)", moving_ispeed)

        # Check separation between standstill noise and motion signal
        p99_noise = p999_ispeed
        p1_motion = percentile(moving_ispeed, 1)
        separation = p1_motion - p99_noise
        print()
        if separation > 0.1:
            print(f"  ✓ GOOD SEPARATION: 99.9th pctile standstill ({p99_noise:.3f} m/s)")
            print(f"    vs 1st pctile moving ({p1_motion:.3f} m/s)")
            print(f"    Gap: {separation:.3f} m/s — threshold placement is robust.")
        else:
            print(f"  ⚠ POOR SEPARATION: 99.9th pctile standstill ({p99_noise:.3f} m/s)")
            print(f"    vs 1st pctile moving ({p1_motion:.3f} m/s)")
            print(f"    Gap: {separation:.3f} m/s — consider using status_speed gate instead.")

    # -------------------------------------------------------------------------
    # STATUS_SPEED vs GPS_SPEED AGREEMENT ANALYSIS
    # -------------------------------------------------------------------------
    print()
    print("-"*70)
    print("GATE SIGNAL QUALITY: status_speed vs gps_speed")
    print("-"*70)
    print()

    # At standstill: how often does gps_speed say non-zero vs status_speed's zero?
    gps_speed_false_pos = sum(1 for s in standstill if s.gps_speed_m_s > 0.05)
    gps_false_rate = gps_speed_false_pos / len(standstill) if standstill else 0.0

    print(f"  At standstill (N={len(standstill)}):")
    print(f"    gps_speed > 0.05 m/s: {gps_speed_false_pos} times ({gps_false_rate:.1%})")
    print(f"    status_speed > 0.05:  0 times  (by definition — this IS the ground truth)")
    print()

    if gps_false_rate > 0.01:
        print("  ⚠ IMPORTANT RECOMMENDATION:")
        print("  ─────────────────────────────────────────────────────────────")
        print("  gps_speed has significant false positives at standstill.")
        print("  The imu_compass node currently gates on gps_speed, which means")
        print("  it will attempt calibration during GPS noise bursts while stopped.")
        print()
        print("  FIX: Add a status_speed subscription to imu_compass_node and use")
        print("  status_speed as the primary speed gate. gps_speed would only be")
        print("  needed to detect speed > 0 — a noisy signal is OK for that.")
        print()
        print("  In imu_compass_node.cpp, change:")
        print()
        print("    // BEFORE (noisy — uses GPS-derived speed):")
        print("    const double speed = latest_speed_m_s_.load();  // from gps_speed_node")
        print("    if (speed < min_calibration_speed_m_s_) { ... }")
        print()
        print("    // AFTER (reliable — uses hardware odometry):")
        print("    const double speed = latest_status_speed_m_s_.load();  // from status_speed")
        print("    if (speed < min_calibration_speed_m_s_) { ... }")
        print()
        print("  This also lets you lower min_calibration_speed_m_s_ closer to")
        print("  the actual minimum operating speed (e.g. 0.3 m/s) rather than")
        print("  having to pad for GPS noise.")
        print("  ─────────────────────────────────────────────────────────────")
    else:
        print("  ✓ gps_speed false positive rate is acceptable.")
        print("    Tuning min_calibration_speed_m_s should be sufficient.")
        print("    (Adding status_speed gate to imu_compass would still be")
        print("    more robust but is not critical given these results.)")

    # -------------------------------------------------------------------------
    # SUMMARY OF RECOMMENDED LAUNCH FILE CHANGES
    # -------------------------------------------------------------------------
    print()
    print("="*70)
    print("RECOMMENDED PARAMETER CHANGES")
    print("="*70)
    print()
    print("  In stack_launch.xml (or gps_speed.launch.py):")
    print(f'    min_distance_m:={rec_min_distance_m:.4f}')
    print()
    print("  In stack_launch.xml (or imu_compass.launch.py):")
    print(f'    min_calibration_speed_m_s:={rec_min_cal_speed:.4f}')
    print()
    print("  These are 99.9th-percentile-with-margin values — statistically,")
    print("  < 0.1% of standstill GPS noise samples will exceed them.")
    print()

    # -------------------------------------------------------------------------
    # CSV DUMP
    # -------------------------------------------------------------------------
    if csv_path:
        _write_csv(csv_path, valid)
        print(f"  Samples written to: {csv_path}")
        print()


def _print_distribution(label: str, data: List[float]):
    """Print distribution statistics for a list of floats."""
    if not data:
        print(f"  {label}: no data")
        return

    print(f"\n  {label}  (N={len(data)})")
    print(f"    Min:   {min(data):.5f}")
    print(f"    Mean:  {statistics.mean(data):.5f}")
    print(f"    Std:   {statistics.stdev(data) if len(data) > 1 else 0.0:.5f}")
    print(f"    P50:   {percentile(data, 50):.5f}")
    print(f"    P90:   {percentile(data, 90):.5f}")
    print(f"    P95:   {percentile(data, 95):.5f}")
    print(f"    P99:   {percentile(data, 99):.5f}")
    print(f"    P99.9: {percentile(data, 99.9):.5f}")
    print(f"    Max:   {max(data):.5f}")


def _write_csv(path: str, samples: List[GpsSample]):
    """Write all samples to a CSV file for offline analysis (e.g. in Excel / Python)."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_s", "lat_deg", "lon_deg", "fix_status",
            "gps_speed_m_s", "status_speed_m_s", "yaw_rate_rad_s",
            "distance_m", "dt_s", "implied_speed_m_s", "is_standstill"
        ])
        for s in samples:
            writer.writerow([
                f"{s.timestamp_s:.3f}",
                f"{s.lat_deg:.9f}", f"{s.lon_deg:.9f}",
                s.fix_status,
                f"{s.gps_speed_m_s:.5f}",
                f"{s.status_speed_m_s:.5f}",
                f"{s.yaw_rate_rad_s:.5f}",
                f"{s.distance_m:.6f}",
                f"{s.dt_s:.4f}",
                f"{s.implied_speed_m_s:.5f}",
                "1" if s.is_standstill else "0"
            ])

#==============================================================================
# MAIN
#==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="GPS noise characterizer for gps_speed / imu_compass tuning")
    parser.add_argument("--robot",  "-r", required=True,
        help="Robot name (e.g. warthog1)")
    parser.add_argument("--duration", "-d", type=float, default=90.0,
        help="Collection duration in seconds (default: 90)")
    parser.add_argument("--phase", choices=["standstill", "moving", "both"],
        default="both",
        help="Collection phase (default: both, auto-classified by status_speed)")
    parser.add_argument("--gps-suffix", default=DEFAULT_GPS_SUFFIX,
        help=f"GPS topic suffix (default: {DEFAULT_GPS_SUFFIX})")
    parser.add_argument("--imu-suffix", default=DEFAULT_IMU_SUFFIX,
        help=f"IMU topic suffix (default: {DEFAULT_IMU_SUFFIX})")
    parser.add_argument("--standstill-thresh", type=float,
        default=DEFAULT_STANDSTILL_THRESHOLD_M_S,
        help="status_speed threshold to classify as standstill (default: 0.05 m/s)")
    parser.add_argument("--csv", metavar="FILE",
        help="Write all samples to a CSV file for offline analysis")
    parser.add_argument("--min-distance-current", type=float, default=0.05,
        help="Current min_distance_m in gps_speed (for comparison)")
    parser.add_argument("--min-cal-speed-current", type=float, default=1.0,
        help="Current min_calibration_speed_m_s in imu_compass (for comparison)")
    args = parser.parse_args()

    rclpy.init()

    collector = GpsNoiseCollector(
        robot_name=args.robot,
        gps_suffix=args.gps_suffix,
        imu_suffix=args.imu_suffix,
        standstill_threshold=args.standstill_thresh,
        duration_s=args.duration,
        phase=args.phase,
        csv_path=args.csv,
    )

    # Spin until collection window closes (or Ctrl+C)
    print(f"  Collecting for {args.duration:.0f}s... (Ctrl+C to stop early)")
    print()

    def _sigint_handler(sig, frame):
        print("\n\n  [Ctrl+C] Stopping collection early...")
        collector.duration_s = 0  # Signal done
    signal.signal(signal.SIGINT, _sigint_handler)

    while not collector.is_done():
        rclpy.spin_once(collector, timeout_sec=0.1)

    print()
    print(f"  Collection complete. Analysing {len(collector.samples)} samples...")

    current_params = {
        "min_distance_m": args.min_distance_current,
        "min_calibration_speed_m_s": args.min_cal_speed_current,
    }

    analyze(
        samples=collector.samples,
        standstill_threshold=args.standstill_thresh,
        current_params=current_params,
        csv_path=args.csv,
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
