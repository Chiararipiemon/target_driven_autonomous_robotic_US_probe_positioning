#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RecordUsApexPoseImFusionCsv (ROS1)

This node records geometry_msgs/PoseStamped messages from a configurable topic and writes them
to a CSV file compatible with ImFusion. Each incoming pose is transformed into a chosen base
frame using TF2, converted to a homogeneous transform in millimeters, then mapped into the
ImFusion coordinate system via a fixed calibration matrix.

What you must change to test this pipeline with your own setup:
1) Input topic:
   - ~topic must publish geometry_msgs/PoseStamped (absolute vs relative name matters).
2) Frames and TF:
   - ~base_frame is the frame you want poses expressed in (e.g., "world").
   - TF must be able to transform msg.header.frame_id -> base_frame.
3) Output:
   - ~output_csv path where the CSV will be saved.
4) Calibration:
   - T_IMFUSION_FROM_ROBOT must match your robot/base_frame to ImFusion coordinate mapping.
     The matrix is expressed in millimeters.
5) Logging control:
   - ~max_samples: set to 0 for unlimited recording.
   - ~flush_every: flush file every N rows to reduce data loss on crashes.

CSV format (no timestamp):
  x_mm, y_mm, z_mm, qx, qy, qz, qw
"""

import os
import math
import csv
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (required for Buffer.transform on PoseStamped)

from geometry_msgs.msg import PoseStamped


# Robot/base_frame -> ImFusion transform in millimeters.
# Replace this matrix with your calibration if you use a different setup.
T_IMFUSION_FROM_ROBOT = np.array(
    [
        [1.0, 0.0, 0.0, -642.0],
        [0.0, 0.0, 1.0, -364.0],
        [0.0, -1.0, 0.0, -200.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.eye(3, dtype=float)
    q /= n
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to quaternion (x, y, z, w)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22

    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


class Recorder:
    def __init__(self):
        rospy.init_node("record_us_apex_pose_imfusion_csv", anonymous=False)

        self.topic = rospy.get_param("~topic", "/us_apex_pose")
        self.base_frame = rospy.get_param("~base_frame", "world")
        self.output_csv = rospy.get_param(
            "~output_csv", "/home/chiararipiemo/validation/us_apex_pose_imfusion.csv"
        )
        self.flush_every = int(rospy.get_param("~flush_every", 1))
        self.max_samples = int(rospy.get_param("~max_samples", 0))

        out_dir = os.path.dirname(self.output_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        self.f = open(self.output_csv, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["x_mm", "y_mm", "z_mm", "qx", "qy", "qz", "qw"])
        self.f.flush()

        self.count = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.on_shutdown(self.on_shutdown)

        self.sub = rospy.Subscriber(self.topic, PoseStamped, self.cb, queue_size=500)

        rospy.loginfo(
            "Recording PoseStamped from '%s' into '%s' (base_frame='%s')",
            self.topic,
            self.output_csv,
            self.base_frame,
        )
        rospy.spin()

    def cb(self, msg: PoseStamped):
        """Transform incoming PoseStamped into base_frame, map to ImFusion frame, and write one CSV row."""
        try:
            msg_base = self.tf_buffer.transform(msg, self.base_frame, rospy.Duration(0.2))
        except Exception:
            try:
                msg2 = PoseStamped()
                msg2.header = msg.header
                msg2.pose = msg.pose
                msg2.header.stamp = rospy.Time(0)
                msg_base = self.tf_buffer.transform(msg2, self.base_frame, rospy.Duration(0.2))
            except Exception as e:
                rospy.logwarn_throttle(
                    1.0,
                    "TF transform failed (%s -> %s): %s",
                    msg.header.frame_id,
                    self.base_frame,
                    str(e),
                )
                return

        p = msg_base.pose.position
        q = msg_base.pose.orientation

        tx, ty, tz = float(p.x), float(p.y), float(p.z)
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)

        R_robot = quat_to_rot(qx, qy, qz, qw)
        t_robot_mm = np.array([tx, ty, tz], dtype=float) * 1000.0

        T_robot_mm = np.eye(4, dtype=float)
        T_robot_mm[:3, :3] = R_robot
        T_robot_mm[:3, 3] = t_robot_mm

        T_if = T_IMFUSION_FROM_ROBOT @ T_robot_mm

        t_if = T_if[:3, 3]
        R_if = T_if[:3, :3]
        q_if = rot_to_quat(R_if)

        self.w.writerow(
            [
                f"{t_if[0]:.6f}",
                f"{t_if[1]:.6f}",
                f"{t_if[2]:.6f}",
                f"{q_if[0]:.9f}",
                f"{q_if[1]:.9f}",
                f"{q_if[2]:.9f}",
                f"{q_if[3]:.9f}",
            ]
        )

        self.count += 1

        if self.flush_every > 0 and (self.count % self.flush_every) == 0:
            self.f.flush()

        if self.max_samples > 0 and self.count >= self.max_samples:
            rospy.loginfo("Reached max_samples=%d. Shutting down.", self.max_samples)
            rospy.signal_shutdown("done")

    def on_shutdown(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass
        rospy.loginfo("Saved %d samples to %s", self.count, self.output_csv)


if __name__ == "__main__":
    try:
        Recorder()
    except rospy.ROSInterruptException:
        pass

