#!/usr/bin/env python3
"""
Logs the robot end-effector pose (position + quaternion) to a CSV file, formatted for ImFusion.
The node reads a TF transform fixed_frame -> ee_frame at a fixed rate, optionally applies a tip offset,
optionally converts position units (m -> mm), and writes rows as: x,y,z,qx,qy,qz,qw.

How to use
- Start the node, then call the services:
  rosservice call /csv_logger/start
  rosservice call /csv_logger/stop

What you may want to tweak
- ~fixed_frame / ~ee_frame: which TF you want to log
- ~rate_hz: logging frequency
- ~output_dir / ~prefix: where and how the CSV is named
- ~units: "mm" (default) or anything else to keep meters
- ~tip_offset_m: if you want the logged point to be offset from the EE frame (e.g., tool tip)
"""

# Code to log the pose information expected by ImFusion (via TF)
import os
import csv
import math
import rospy
import numpy as np
import tf2_ros
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix

class CsvLogger:
    def __init__(self):
        # Main parameters
        self.fixed_frame  = rospy.get_param("~fixed_frame", "world")        # TF parent frame
        self.ee_frame     = rospy.get_param("~ee_frame", "iiwa_link_ee")    # TF child frame (end-effector)
        self.rate_hz      = float(rospy.get_param("~rate_hz", 20.0))        # logging rate
        self.units        = rospy.get_param("~units", "mm")                # "mm" to convert meters -> millimeters
        self.tip_offset_m = float(rospy.get_param("~tip_offset_m", 0.0))   # optional tip offset in meters
        self.output_dir   = os.path.expanduser(rospy.get_param("~output_dir", "~/iiwa_csv"))
        self.prefix       = rospy.get_param("~prefix", "us_poses")

        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)

        # TF listener (buffer keeps a small history to tolerate minor delays)
        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        # Recording state
        self.is_recording = False
        self.csv_file = None
        self.writer = None

        # Services to start/stop logging (under the node namespace)
        self.srv_start = rospy.Service("~start", Trigger, self._start_cb)
        self.srv_stop  = rospy.Service("~stop",  Trigger, self._stop_cb)

        rospy.loginfo("[csv_logger] Ready. Call ~start to begin recording, ~stop to end.")

    def _start_cb(self, _req):
        # Open a new CSV file and start writing rows
        if self.is_recording:
            return TriggerResponse(success=False, message="Already recording. Stop first.")

        fname = f"{self.prefix}_{rospy.Time.now().to_sec():.0f}.csv"
        path = os.path.join(self.output_dir, fname)

        try:
            self.csv_file = open(path, "w", newline="")
            self.writer = csv.writer(self.csv_file)

            # Header matches the format expected by downstream tools
            self.writer.writerow(["x","y","z","qx","qy","qz","qw"])
            self.csv_file.flush()

            self.is_recording = True
            rospy.loginfo(f"[csv_logger] Recording ON -> {path}")
            return TriggerResponse(success=True, message=f"Recording to {path}")

        except Exception as e:
            return TriggerResponse(success=False, message=f"Failed to open file: {e}")

    def _stop_cb(self, _req):
        # Stop recording and close the CSV file
        if not self.is_recording:
            return TriggerResponse(success=False, message="Not recording.")

        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.writer = None
            self.is_recording = False
            rospy.loginfo("[csv_logger] Recording OFF.")
            return TriggerResponse(success=True, message="Recording stopped")

        except Exception as e:
            return TriggerResponse(success=False, message=f"Failed to close file: {e}")

    def _lookup(self):
        """
        Lookup TF fixed_frame -> ee_frame and return (x,y,z,qx,qy,qz,qw).

        Position is read in meters from TF, then:
        - optional tip offset is applied along the tool -Z axis (in the ee_frame orientation)
        - optional unit conversion to millimeters is applied
        """
        try:
            tf: TransformStamped = self.tf_buf.lookup_transform(
                self.fixed_frame, self.ee_frame, rospy.Time(0), rospy.Duration(0.2)
            )

            # Translation (meters) and rotation (quaternion) from TF
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

            # Optional tip offset: shift the logged point along -Z of the tool frame.
            # This is useful if ee_frame is at the flange but you want the probe tip position.
            if self.tip_offset_m != 0.0:
                R = quaternion_matrix([qx, qy, qz, qw])[:3, :3]
                offset_tool  = np.array([0.0, 0.0, -self.tip_offset_m])  # along -Z in ee_frame
                offset_fixed = R.dot(offset_tool)                         # expressed in fixed_frame
                tx += float(offset_fixed[0])
                ty += float(offset_fixed[1])
                tz += float(offset_fixed[2])

            # Unit conversion (position only). Rotation stays a unit quaternion.
            if self.units.lower() == "mm":
                s = 1000.0
            else:
                s = 1.0

            return (tx*s, ty*s, tz*s, qx, qy, qz, qw)

        except TransformException as e:
            # Throttle warnings to avoid spamming the console when TF is temporarily missing
            rospy.logwarn_throttle(1.0, f"[csv_logger] TF not available: {e}")
            return None

    def spin(self):
        # Main loop: when recording, query TF and append a formatted CSV row
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.is_recording and self.writer is not None:
                data = self._lookup()
                if data is not None:
                    self.writer.writerow([
                        f"{data[0]:.6f}", f"{data[1]:.6f}", f"{data[2]:.6f}",
                        f"{data[3]:.6f}", f"{data[4]:.6f}", f"{data[5]:.6f}", f"{data[6]:.6f}"
                    ])
                    # Flush frequently for safety (at the cost of some I/O overhead)
                    self.csv_file.flush()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("csv_logger", anonymous=False)
    CsvLogger().spin()
