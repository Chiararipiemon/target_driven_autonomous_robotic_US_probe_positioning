#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Description
Logs end-effector poses to a CSV file in the format:
  x,y,z,qx,qy,qz,qw

Data is read from TF (fixed_frame -> ee_frame) at a given rate. Optionally, a tip offset
can be applied along the tool -Z axis (in tool frame) before logging.

Usage
  rosrun iiwa_probe_utils csv_logger.py _fixed_frame:=world _ee_frame:=probe_tip

Services
  ~start  (std_srvs/Trigger): open a new CSV and start logging
  ~stop   (std_srvs/Trigger): stop logging and close the CSV

What to change for your own setup
- Frames:
    * fixed_frame (e.g., world/map) and ee_frame (e.g., probe_tip/tool0/iiwa_link_ee)
- Units:
    * units: "mm" (default) or "m"
- Output:
    * output_dir and prefix
- Tip reference:
    * tip_offset_m (meters, along tool -Z)
- Performance:
    * flush_every_n (flush frequency to reduce disk I/O)
"""

import os
import csv
import rospy
import numpy as np
import tf2_ros
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix


class CsvLogger:
    def __init__(self):
        # --- params ---
        self.fixed_frame = rospy.get_param("~fixed_frame", "world")
        self.ee_frame = rospy.get_param("~ee_frame", "probe_tip")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.units = str(rospy.get_param("~units", "mm")).lower()
        self.tip_offset_m = float(rospy.get_param("~tip_offset_m", 0.0))
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/iiwa_csv"))
        self.prefix = rospy.get_param("~prefix", "us_poses")
        self.flush_every_n = int(rospy.get_param("~flush_every_n", 1))  # 1 = flush each row (safe, slower)

        os.makedirs(self.output_dir, exist_ok=True)

        # --- TF ---
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        # --- state ---
        self.is_recording = False
        self.csv_file = None
        self.writer = None
        self.row_count = 0
        self.current_path = ""

        # --- services ---
        self.srv_start = rospy.Service("~start", Trigger, self._start_cb)
        self.srv_stop = rospy.Service("~stop", Trigger, self._stop_cb)

        rospy.on_shutdown(self._shutdown_cb)

        rospy.loginfo("[csv_logger] Ready. Call ~start to record, ~stop to finish.")
        rospy.loginfo("[csv_logger] TF: %s -> %s | rate=%.1f Hz | units=%s | out=%s",
                      self.fixed_frame, self.ee_frame, self.rate_hz, self.units, self.output_dir)

    def _shutdown_cb(self):
        # Ensure file is closed on shutdown.
        try:
            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass

    def _start_cb(self, _req):
        if self.is_recording:
            return TriggerResponse(success=False, message="Already recording.")

        # Unique filename based on ROS time in nanoseconds.
        stamp_ns = rospy.Time.now().to_nsec()
        fname = f"{self.prefix}_{stamp_ns}.csv"
        self.current_path = os.path.join(self.output_dir, fname)

        try:
            self.csv_file = open(self.current_path, "w", newline="")
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(["x", "y", "z", "qx", "qy", "qz", "qw"])
            self.csv_file.flush()
            self.row_count = 0
            self.is_recording = True
            rospy.loginfo("[csv_logger] Recording ON -> %s", self.current_path)
            return TriggerResponse(success=True, message=f"Recording to {self.current_path}")
        except Exception as e:
            self.csv_file = None
            self.writer = None
            self.is_recording = False
            return TriggerResponse(success=False, message=f"Failed to open CSV: {e}")

    def _stop_cb(self, _req):
        if not self.is_recording:
            return TriggerResponse(success=False, message="Not recording.")

        try:
            self.csv_file.flush()
            self.csv_file.close()
            path = self.current_path
            rows = self.row_count

            self.csv_file = None
            self.writer = None
            self.is_recording = False
            self.current_path = ""

            rospy.loginfo("[csv_logger] Recording OFF. Saved %d rows -> %s", rows, path)
            return TriggerResponse(success=True, message=f"Saved {rows} rows to {path}")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Failed to close CSV: {e}")

    def _lookup_pose(self):
        # Lookup latest TF from fixed_frame to ee_frame.
        try:
            tf: TransformStamped = self.tf_buf.lookup_transform(
                self.fixed_frame, self.ee_frame, rospy.Time(0), rospy.Duration(0.2)
            )

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            # Optional tip offset along tool -Z axis (meters).
            if abs(self.tip_offset_m) > 0.0:
                R = quaternion_matrix([qx, qy, qz, qw])[:3, :3]
                offset_tool = np.array([0.0, 0.0, -self.tip_offset_m], dtype=float)
                offset_world = R.dot(offset_tool)
                tx += float(offset_world[0])
                ty += float(offset_world[1])
                tz += float(offset_world[2])

            # Unit conversion (positions only).
            if self.units == "mm":
                scale = 1000.0
            elif self.units == "m":
                scale = 1.0
            else:
                rospy.logwarn_throttle(5.0, "[csv_logger] Unknown units='%s' (using meters).", self.units)
                scale = 1.0

            return (tx * scale, ty * scale, tz * scale, qx, qy, qz, qw)

        except TransformException as e:
            rospy.logwarn_throttle(1.0, "[csv_logger] TF unavailable: %s", str(e))
            return None

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.is_recording and self.writer is not None:
                data = self._lookup_pose()
                if data is not None:
                    self.writer.writerow([
                        f"{data[0]:.6f}", f"{data[1]:.6f}", f"{data[2]:.6f}",
                        f"{data[3]:.6f}", f"{data[4]:.6f}", f"{data[5]:.6f}", f"{data[6]:.6f}"
                    ])
                    self.row_count += 1

                    # Reduce disk I/O if desired.
                    if self.flush_every_n <= 1 or (self.row_count % self.flush_every_n) == 0:
                        self.csv_file.flush()
            rate.sleep()


def main():
    rospy.init_node("csv_logger", anonymous=False)
    CsvLogger().spin()


if __name__ == "__main__":
    main()
