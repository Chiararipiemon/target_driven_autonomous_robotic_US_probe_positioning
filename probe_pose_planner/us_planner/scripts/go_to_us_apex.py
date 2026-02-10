#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GoToUsApex (ROS1 / MoveIt)

This node moves a robot tool tip to an ultrasound apex pose (PoseStamped) using MoveIt.
It assumes you plan on an end-effector link (ee_link) but you want the final goal to match
a tool tip frame (tip_frame). The node uses TF to compute the rigid transform ee_link -> tip_frame
and converts a desired tip pose into an equivalent ee_link goal pose for MoveIt.

What you must change to test this pipeline with your own setup:
1) MoveIt planning group and links:
   - ~group_name: your MoveIt group (e.g. "manipulator")
   - ~ee_link: the MoveIt link used as end-effector for planning
2) Frames and TF:
   - ~ref_frame: planning reference frame (commonly "world" or "base_link")
   - ~tip_frame: your tool tip TF frame
   - TF must provide a valid transform: ee_link -> tip_frame
3) Apex input topic:
   - ~apex_pose_topic: topic publishing geometry_msgs/PoseStamped for the apex goal
     If your pipeline publishes a different message (e.g. a Point), you must convert it
     to PoseStamped or extend this node accordingly.
4) Robot configurations:
   - ~pre_joints and ~target_joints: joint arrays matching your robot joint order
5) Performance/tolerances:
   - speed scaling, planning time/attempts, position/orientation tolerances, refine loop thresholds

Typical usage:
- Start your robot drivers + robot_state_publisher + TF tree
- Start MoveIt move_group in the same namespace as this node (or adjust robot_description lookup)
- Publish a PoseStamped apex on ~apex_pose_topic
- Launch this node via roslaunch and parameters YAML
"""

import sys
import copy
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft

# Requires ROS package: tf2_geometry_msgs (Noetic: ros-noetic-tf2-geometry-msgs)
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander


def norm(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def pose_from(p, q):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, p)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
    return msg


def mat_from_pose(p, q):
    T = tft.quaternion_matrix(q)
    T[0:3, 3] = np.array(p, dtype=float).reshape(3,)
    return T


def pose_from_mat(T):
    p = T[0:3, 3].copy()
    q = tft.quaternion_from_matrix(T)
    return p, q


def angle_deg_from_R(R):
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))


def quat_normalize(q):
    q = np.array(q, dtype=float).reshape(4,)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return None
    return q / n


def quat_continuity(q_des, q_ref):
    """Keeps quaternion sign continuity: if dot < 0, flip desired quaternion."""
    if q_des is None or q_ref is None:
        return q_des
    return -q_des if float(np.dot(q_des, q_ref)) < 0.0 else q_des


class GoToUsApex(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("go_to_us_apex")

        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.ee_link = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame = rospy.get_param("~ref_frame", "world")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.2))

        self.pre_joints = rospy.get_param(
            "~pre_joints",
            [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174],
        )
        self.target_joints = rospy.get_param(
            "~target_joints",
            [-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169],
        )

        self.apex_pose_topic = rospy.get_param("~apex_pose_topic", "us_apex_pose")
        self.apex_timeout = float(rospy.get_param("~apex_timeout", 10.0))

        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")
        self.contact_margin = float(rospy.get_param("~contact_margin", 0.0))

        self.planning_time = float(rospy.get_param("~planning_time", 15.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 20))
        self.pos_tol = float(rospy.get_param("~pos_tol", 0.001))
        self.ori_tol_deg = float(rospy.get_param("~ori_tol_deg", 2.0))

        self.max_refine = int(rospy.get_param("~max_refine", 3))
        self.tip_pos_threshold_mm = float(rospy.get_param("~tip_pos_threshold_mm", 1.0))
        self.tip_ori_threshold_deg = float(rospy.get_param("~tip_ori_threshold_deg", 0.5))

        self.use_tip_based_ori_tol = bool(rospy.get_param("~use_tip_based_ori_tol", True))
        self.desired_tip_mm = float(rospy.get_param("~desired_tip_mm", 1.0))

        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"

        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)

        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_end_effector_link(self.ee_link)

        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)

        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.set_goal_position_tolerance(self.pos_tol)
        self.group.set_goal_orientation_tolerance(np.deg2rad(self.ori_tol_deg))
        self.group.allow_replanning(True)

        self.joint_names = self.group.get_active_joints()

        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        rospy.loginfo(
            "Planning on ee_link='%s' using tip_frame='%s' in ref_frame='%s'",
            self.ee_link, self.tip_frame, self.ref_frame
        )

        self.T_ee_tip = self.lookup_T(self.ee_link, self.tip_frame, timeout=5.0)
        self.T_tip_ee = np.linalg.inv(self.T_ee_tip)

        t = self.T_ee_tip[0:3, 3]
        self.tip_offset = float(np.linalg.norm(t))
        rospy.loginfo(
            "TF %s -> %s translation [%.4f %.4f %.4f] m, norm=%.4f m",
            self.ee_link, self.tip_frame, t[0], t[1], t[2], self.tip_offset
        )

        if self.use_tip_based_ori_tol and self.tip_offset > 1e-6:
            max_ori_rad = (self.desired_tip_mm / 1000.0) / self.tip_offset
            max_ori_deg = float(np.degrees(max_ori_rad))
            new_ori_deg = float(min(self.ori_tol_deg, max_ori_deg))
            new_ori_deg = max(new_ori_deg, 0.1)
            self.group.set_goal_orientation_tolerance(np.deg2rad(new_ori_deg))
            rospy.loginfo(
                "Orientation tolerance set to %.3f deg (user=%.3f deg, tip-based=%.3f deg, floor=0.1 deg)",
                new_ori_deg, self.ori_tol_deg, max_ori_deg
            )

        self.run_sequence()

    def lookup_T(self, target_frame, source_frame, timeout=2.0):
        """Returns a 4x4 transform matrix T_target_source (target <- source)."""
        ok = self.tf_buf.can_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
        )
        if not ok:
            raise rospy.ROSException(f"TF not available: {target_frame} <- {source_frame}")

        tfm = self.tf_buf.lookup_transform(
            target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout)
        )
        tx = tfm.transform.translation.x
        ty = tfm.transform.translation.y
        tz = tfm.transform.translation.z
        qx = tfm.transform.rotation.x
        qy = tfm.transform.rotation.y
        qz = tfm.transform.rotation.z
        qw = tfm.transform.rotation.w

        T = tft.quaternion_matrix([qx, qy, qz, qw])
        T[0:3, 3] = [tx, ty, tz]
        return T

    def transform_pose_to_ref(self, pose_msg, timeout=0.5):
        """Transforms a PoseStamped into self.ref_frame using TF2."""
        if not pose_msg.header.frame_id:
            pose_msg = copy.deepcopy(pose_msg)
            pose_msg.header.frame_id = self.ref_frame

        try:
            return self.tf_buf.transform(pose_msg, self.ref_frame, rospy.Duration(timeout))
        except Exception:
            try:
                msg2 = copy.deepcopy(pose_msg)
                msg2.header.stamp = rospy.Time(0)
                return self.tf_buf.transform(msg2, self.ref_frame, rospy.Duration(timeout))
            except Exception as e2:
                rospy.logerr("TF transform to %s failed: %s", self.ref_frame, str(e2))
                return None

    def go_to_joints(self, joint_vals, tag="target"):
        jd = {}
        for i in range(len(joint_vals)):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)

        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if ok:
            rospy.loginfo("Reached '%s' via planner.", tag)
        else:
            rospy.logwarn("Planner failed to reach '%s'.", tag)
        return ok

    def get_T_world_ee(self):
        cur = self.group.get_current_pose(self.ee_link).pose
        p = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q = quat_normalize([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w])
        if q is None:
            q = np.array([0.0, 0.0, 0.0, 1.0], float)
        return mat_from_pose(p, q)

    def compute_tip_pose_from_world_ee(self, T_world_ee):
        T_world_tip = T_world_ee @ self.T_ee_tip
        p_tip = T_world_tip[0:3, 3].copy()
        q_tip = quat_normalize(tft.quaternion_from_matrix(T_world_tip))
        return p_tip, q_tip, T_world_tip

    def run_sequence(self):
        self.go_to_joints(self.pre_joints, "pre_approach")
        self.go_to_joints(self.target_joints, "target_pose")

        rospy.loginfo(
            "Waiting for PoseStamped apex on topic '%s' (timeout=%.1f s)...",
            self.apex_pose_topic, self.apex_timeout
        )
        try:
            apex_pose_msg = rospy.wait_for_message(
                self.apex_pose_topic, PoseStamped, timeout=self.apex_timeout
            )
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for apex pose on '%s'.", self.apex_pose_topic)
            return

        self.move_tip_to_apex(apex_pose_msg)

    def move_tip_to_apex(self, apex_pose_msg):
        apex_pose_ref = self.transform_pose_to_ref(apex_pose_msg, timeout=0.5)
        if apex_pose_ref is None:
            return

        apex = np.array(
            [
                apex_pose_ref.pose.position.x,
                apex_pose_ref.pose.position.y,
                apex_pose_ref.pose.position.z,
            ],
            float,
        )

        q_tip_goal = quat_normalize(
            [
                apex_pose_ref.pose.orientation.x,
                apex_pose_ref.pose.orientation.y,
                apex_pose_ref.pose.orientation.z,
                apex_pose_ref.pose.orientation.w,
            ]
        )
        if q_tip_goal is None:
            rospy.logerr("Invalid apex quaternion (norm ~ 0).")
            return

        T_world_ee_now = self.get_T_world_ee()
        _, q_tip_now, _ = self.compute_tip_pose_from_world_ee(T_world_ee_now)
        q_tip_goal = quat_continuity(q_tip_goal, q_tip_now)

        R_tip_goal = tft.quaternion_matrix(q_tip_goal)[:3, :3]
        z_tool = norm(R_tip_goal[:, 2])

        tip_goal_pos = apex - self.contact_margin * z_tool
        T_world_tip_goal = mat_from_pose(tip_goal_pos, q_tip_goal)

        rospy.loginfo(
            "Tip goal in '%s': pos=[%.4f %.4f %.4f], contact_margin=%.4f",
            self.ref_frame, tip_goal_pos[0], tip_goal_pos[1], tip_goal_pos[2], self.contact_margin
        )

        refine_iters = max(1, self.max_refine)
        for k in range(refine_iters):
            T_world_ee_goal = T_world_tip_goal @ self.T_tip_ee
            ee_goal_p, ee_goal_q = pose_from_mat(T_world_ee_goal)
            ee_goal_q = quat_normalize(ee_goal_q)
            if ee_goal_q is None:
                ee_goal_q = np.array([0.0, 0.0, 0.0, 1.0], float)

            pose_goal = pose_from(ee_goal_p, ee_goal_q)

            T_world_ee_now = self.get_T_world_ee()
            tip_now_p, _, _ = self.compute_tip_pose_from_world_ee(T_world_ee_now)

            rospy.loginfo(
                "[refine %d/%d] tip_now=[%.4f %.4f %.4f], apex=[%.4f %.4f %.4f]",
                k + 1, refine_iters,
                tip_now_p[0], tip_now_p[1], tip_now_p[2],
                apex[0], apex[1], apex[2]
            )

            self.group.set_pose_reference_frame(self.ref_frame)
            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(pose_goal, self.ee_link)

            ok = self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            T_world_ee_after = self.get_T_world_ee()
            _, _, T_world_tip_after = self.compute_tip_pose_from_world_ee(T_world_ee_after)

            tip_err_mm = float(np.linalg.norm(T_world_tip_after[0:3, 3] - tip_goal_pos) * 1000.0)

            R_now_tip = T_world_tip_after[0:3, 0:3]
            R_goal_tip = T_world_tip_goal[0:3, 0:3]
            R_err = R_goal_tip.T @ R_now_tip
            ori_err_deg = angle_deg_from_R(R_err)

            rospy.loginfo(
                "[refine %d/%d] ok=%s tip_err=%.2f mm ori_err=%.2f deg",
                k + 1, refine_iters, str(ok), tip_err_mm, ori_err_deg
            )

            if not ok:
                rospy.logerr("Planning/execution failed at refine %d. Stopping.", k + 1)
                return

            if (tip_err_mm <= self.tip_pos_threshold_mm) and (ori_err_deg <= self.tip_ori_threshold_deg):
                rospy.loginfo(
                    "Success: tip within thresholds (<= %.2f mm, <= %.2f deg).",
                    self.tip_pos_threshold_mm, self.tip_ori_threshold_deg
                )
                return

        rospy.logwarn(
            "Refine loop finished without meeting thresholds. Consider tightening ori_tol_deg or increasing max_refine."
        )


if __name__ == "__main__":
    try:
        GoToUsApex()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            roscpp_shutdown()
        except Exception:
            pass

