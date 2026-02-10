#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
go_to_us_best_pose.py

Move the robot through a joint pre-configuration, then a target/vantage joint configuration,
then approach a best probe pose published as geometry_msgs/PoseStamped (typically /us_best_pose).

The incoming PoseStamped is interpreted as:
- position: desired skin contact point s* in ref_frame
- orientation: desired tool orientation (Z axis is the desired probe forward axis)

The node computes the rigid offset from the MoveIt end-effector link to the probe tip frame
using TF (ee_link -> tip_frame). It then commands the end-effector pose such that:

  tip_position = s* - contact_margin * Z_tool

Optionally, a two-stage approach is used:
1) pre-contact: tip_pre  = s* - (contact_margin + approach_dist) * Z_tool
2) contact:     tip_goal = s* - contact_margin * Z_tool

What you must change to test this pipeline with your own setup:
1) MoveIt configuration:
   - ~group_name: planning group name
   - ~ee_link: end-effector link used for planning
   - ~ref_frame: planning reference frame (world/base_link)
2) Robot joint configurations:
   - ~pre_joints and ~target_joints must match your robot joint order
3) Tool geometry and TF:
   - ~tip_frame must exist in TF and be connected to ~ee_link
   - If TF is not available, set ~tip_to_contact as a fallback distance along +Z_EE
4) Input topic:
   - ~best_pose_topic must publish geometry_msgs/PoseStamped
5) Approach parameters:
   - ~contact_margin and ~approach_dist are expressed in meters
"""

import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander


def norm(v, eps=1e-12):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def pose_from(p, q):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, p)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
    return msg


class GoToUsBestPose(object):
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("go_to_us_best_pose")

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

        self.best_pose_topic = rospy.get_param("~best_pose_topic", "/us_best_pose")
        self.best_pose_timeout = float(rospy.get_param("~best_pose_timeout", 10.0))

        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))
        self.contact_margin = float(rospy.get_param("~contact_margin", 0.006))

        self.two_stage = bool(rospy.get_param("~two_stage", True))
        self.approach_dist = float(rospy.get_param("~approach_dist", 0.03))

        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)

        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)

        self.joint_names = self.group.get_active_joints()

        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        self.tip_vec_ee = self.compute_tip_vector()
        rospy.loginfo("Tip vector EE->%s: [%.4f %.4f %.4f] m", self.tip_frame,
                      self.tip_vec_ee[0], self.tip_vec_ee[1], self.tip_vec_ee[2])

        self.run_sequence()

    def compute_tip_vector(self):
        """Returns the translation vector from ee_link to tip_frame expressed in ee_link coordinates."""
        try:
            tfm = self.tf_buf.lookup_transform(
                self.ee_link, self.tip_frame, rospy.Time(0), rospy.Duration(2.0)
            )
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y
            tz = tfm.transform.translation.z
            rospy.loginfo("TF %s -> %s translation: [%.4f %.4f %.4f] m", self.ee_link, self.tip_frame, tx, ty, tz)
            return np.array([tx, ty, tz], float)
        except Exception as e:
            rospy.logwarn(
                "TF lookup failed (%s -> %s): %s. Using fallback tip_to_contact=%.4f m along +Z_EE.",
                self.ee_link,
                self.tip_frame,
                str(e),
                self.tip_to_contact,
            )
            return np.array([0.0, 0.0, float(self.tip_to_contact)], float)

    def go_to_joints(self, joint_vals, tag="target"):
        """Moves the robot to a joint configuration."""
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

        rospy.loginfo("%s -> %s", tag, "OK" if ok else "FAILED")
        return bool(ok)

    def go_to_pose(self, pose_goal, tag="pose_goal"):
        """Plans and executes a MoveIt motion to a pose goal."""
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose_goal, self.ee_link)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        rospy.loginfo("%s -> %s", tag, "OK" if ok else "FAILED")
        return bool(ok)

    def run_sequence(self):
        self.go_to_joints(self.pre_joints, "pre_approach")
        self.go_to_joints(self.target_joints, "target_pose")

        rospy.loginfo(
            "Waiting for best pose on topic '%s' (timeout=%.1f s)...",
            self.best_pose_topic,
            self.best_pose_timeout,
        )
        try:
            best_msg = rospy.wait_for_message(self.best_pose_topic, PoseStamped, timeout=self.best_pose_timeout)
        except rospy.ROSException:
            rospy.logerr("Timeout while waiting for best pose on '%s'.", self.best_pose_topic)
            return

        best_msg = self.ensure_pose_in_ref_frame(best_msg)
        if best_msg is None:
            rospy.logerr("Cannot transform best pose into ref_frame='%s'.", self.ref_frame)
            return

        self.move_tip_to_best_pose(best_msg)

    def ensure_pose_in_ref_frame(self, pose_st):
        """Transforms pose_st into ref_frame if needed."""
        src = pose_st.header.frame_id
        if src == self.ref_frame or src == "":
            return pose_st
        try:
            tfm = self.tf_buf.lookup_transform(self.ref_frame, src, rospy.Time(0), rospy.Duration(2.0))
            out = tf2_geometry_msgs.do_transform_pose(pose_st, tfm)
            out.header.frame_id = self.ref_frame
            return out
        except Exception as e:
            rospy.logwarn("TF transform failed (%s -> %s): %s", src, self.ref_frame, str(e))
            return None

    def move_tip_to_best_pose(self, best_pose_msg):
        """Computes EE poses for pre-contact/contact and executes the motion."""
        s_star = np.array(
            [
                best_pose_msg.pose.position.x,
                best_pose_msg.pose.position.y,
                best_pose_msg.pose.position.z,
            ],
            float,
        )

        q_goal = np.array(
            [
                best_pose_msg.pose.orientation.x,
                best_pose_msg.pose.orientation.y,
                best_pose_msg.pose.orientation.z,
                best_pose_msg.pose.orientation.w,
            ],
            float,
        )
        qn = np.linalg.norm(q_goal)
        if qn < 1e-9:
            rospy.logwarn("Best pose quaternion is invalid. Using current EE orientation.")
            cur = self.group.get_current_pose(self.ee_link).pose
            q_goal = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        else:
            q_goal = q_goal / qn

        R_goal = tft.quaternion_matrix(q_goal)[:3, :3]
        z_tool = norm(R_goal[:, 2])

        tip_goal = s_star - self.contact_margin * z_tool
        tip_pre = s_star - (self.contact_margin + max(0.0, self.approach_dist)) * z_tool

        ee_goal = tip_goal - R_goal.dot(self.tip_vec_ee)
        ee_pre = tip_pre - R_goal.dot(self.tip_vec_ee)

        rospy.loginfo("Best contact point s*: [%.4f %.4f %.4f] (ref_frame=%s)", s_star[0], s_star[1], s_star[2], self.ref_frame)
        rospy.loginfo("Best Z_tool axis:        [%.4f %.4f %.4f]", z_tool[0], z_tool[1], z_tool[2])
        rospy.loginfo("Tip pre-contact:         [%.4f %.4f %.4f]", tip_pre[0], tip_pre[1], tip_pre[2])
        rospy.loginfo("Tip contact goal:        [%.4f %.4f %.4f]", tip_goal[0], tip_goal[1], tip_goal[2])
        rospy.loginfo("EE pre-contact:          [%.4f %.4f %.4f]", ee_pre[0], ee_pre[1], ee_pre[2])
        rospy.loginfo("EE contact goal:         [%.4f %.4f %.4f]", ee_goal[0], ee_goal[1], ee_goal[2])

        if self.two_stage and self.approach_dist > 0.0:
            pose_pre = pose_from(ee_pre, q_goal)
            if not self.go_to_pose(pose_pre, "best_pre_contact"):
                rospy.logerr("Pre-contact motion toward best pose failed.")
                return

        pose_goal = pose_from(ee_goal, q_goal)
        if not self.go_to_pose(pose_goal, "best_contact"):
            rospy.logerr("Contact motion toward best pose failed.")
            return

        rospy.loginfo("Reached best pose vicinity (contact_margin=%.4f m).", self.contact_margin)


if __name__ == "__main__":
    try:
        GoToUsBestPose()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            roscpp_shutdown()
        except Exception:
            pass

