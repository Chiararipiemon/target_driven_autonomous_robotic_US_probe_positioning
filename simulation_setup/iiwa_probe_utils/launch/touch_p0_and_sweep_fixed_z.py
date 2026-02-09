#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Description
Touch + linear sweep on a point-cloud surface while keeping a FIXED probe orientation.

Pipeline:
1) Move to pre_approach joint pose (planner, with joint-interp fallback).
2) Move to target joint pose (planner, with joint-interp fallback).
3) Pick a contact point P0 from the input point cloud (closest point to the ray
   starting at the probe tip and going forward along -Z_tool).
4) Touch P0 with Z_tool aligned to a fixed direction in ref_frame (fixed_z_dir).
5) Generate a linear sweep path on the surface by sampling along a preferred
   direction (sweep_pref_dir) projected onto the plane orthogonal to fixed_z_dir,
   snapping each sample to the nearest cloud point.
6) Retract away from the surface along -Z_tool (opposite of fixed Z).
7) Optionally return to pre_approach.

What to change for your own setup
- pre_joints / target_joints: your preferred approach poses
- cloud_topic: your point cloud topic (e.g., /skin_cloud)
- fixed_z_dir: desired probe Z axis direction in ref_frame (e.g., [0,0,-1])
- sweep_length / sweep_samples / sweep_pref_dir: sweep geometry
- contact_margin / approach_dist / retreat_dist: contact safety and approach/retract distances
- ik_service: usually /<robot_name>/compute_ik
"""

import sys
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


def unit(v, eps=1e-12):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def pose_from(p, q):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = float(p[0]), float(p[1]), float(p[2])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.tolist()
    return msg


def quat_from_axes(x, y, z):
    R = np.column_stack((unit(x), unit(y), unit(z)))
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return np.array([qx, qy, qz, qw], float)


def quat_from_z_min_yaw(q_prev, z_des):
    # Align Z axis to z_des while keeping yaw close to q_prev.
    Rprev = tft.quaternion_matrix(q_prev)[:3, :3]
    x_prev = Rprev[:, 0]
    z = unit(z_des)

    x_proj = x_prev - np.dot(x_prev, z) * z
    if np.linalg.norm(x_proj) < 1e-6:
        up = np.array([0.0, 0.0, 1.0])
        x_proj = up - np.dot(up, z) * z
        if np.linalg.norm(x_proj) < 1e-6:
            up = np.array([1.0, 0.0, 0.0])
            x_proj = up - np.dot(up, z) * z

    x = unit(x_proj)
    y = unit(np.cross(z, x))
    x = unit(np.cross(y, z))
    return quat_from_axes(x, y, z)


def slerp(q0, q1, s):
    return np.array(tft.quaternion_slerp(q0, q1, float(s)), float)


class TouchAndSweepFixedZ:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("touch_p0_and_sweep_fixed_z", anonymous=False)

        # --- MoveIt config ---
        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.ee_link = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame = rospy.get_param("~ref_frame", "world")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.2))

        self.pre_joints = rospy.get_param("~pre_joints", [])
        self.target_joints = rospy.get_param("~target_joints", [])

        self.fallback_steps_pre = int(rospy.get_param("~fallback_steps_pre", 30))
        self.fallback_steps_tgt = int(rospy.get_param("~fallback_steps_tgt", 40))
        self.fallback_dt = float(rospy.get_param("~fallback_dt", 0.2))

        # --- Tip frames ---
        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))

        # --- Cloud ---
        self.cloud_topic = rospy.get_param("~cloud_topic", "/skin_cloud")
        self.wait_for_cloud_sec = float(rospy.get_param("~wait_for_cloud_sec", 30.0))

        # --- Touch ---
        self.contact_margin = float(rospy.get_param("~contact_margin", 0.006))
        self.approach_dist = float(rospy.get_param("~approach_dist", 0.03))
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 0.06))

        self.far_steps = int(rospy.get_param("~far_steps", 1))
        self.pre_steps = int(rospy.get_param("~pre_steps", 12))
        self.approach_steps = int(rospy.get_param("~approach_steps", 18))

        self.touch_step_time = float(rospy.get_param("~step_time", 0.25))

        # --- Sweep ---
        self.sweep_length = float(rospy.get_param("~sweep_length", 0.20))
        self.sweep_samples = int(rospy.get_param("~sweep_samples", 40))
        self.sweep_step_time = float(rospy.get_param("~sweep_step_time", 0.25))
        self.sweep_pref_dir = unit(rospy.get_param("~sweep_pref_dir", [0.0, 1.0, 0.0]))
        self.sweep_contact_margin = float(rospy.get_param("~sweep_contact_margin", self.contact_margin))

        # --- Fixed orientation ---
        self.use_fixed_z = bool(rospy.get_param("~use_fixed_z", True))
        self.fixed_z_dir = unit(rospy.get_param("~fixed_z_dir", [0.0, 0.0, -1.0]))
        self.fixed_flip_to_face_target = bool(rospy.get_param("~fixed_flip_to_face_target", True))

        # --- IK ---
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 3.0))
        self.ik_service_param = rospy.get_param("~ik_service", "")
        self.allow_partial = bool(rospy.get_param("~allow_partial", True))
        self.min_partial_fraction = float(rospy.get_param("~min_partial_fraction", 0.10))

        # --- Behavior toggles ---
        self.return_to_pre = bool(rospy.get_param("~return_to_pre", True))
        self.execute = bool(rospy.get_param("~execute", True))

        # --- MoveIt init ---
        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.joint_names = self.group.get_active_joints()

        # --- TF ---
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
        self.tip_vec_ee = self._compute_tip_vector_ee()

        # --- IK service handle ---
        self.ik_srv = None

        rospy.loginfo("[fixedZ] Ready: cloud=%s | fixed_z=%s | sweep_len=%.3f",
                      self.cloud_topic, self.fixed_z_dir.tolist(), self.sweep_length)

        # --- Run pipeline ---
        self._go_to_joints(self.pre_joints, tag="pre_approach", fallback_steps=self.fallback_steps_pre)
        self._go_to_joints(self.target_joints, tag="target_pose", fallback_steps=self.fallback_steps_tgt)

        self._touch_sweep_retract()

        if self.return_to_pre:
            self._go_to_joints(self.pre_joints, tag="pre_approach", fallback_steps=self.fallback_steps_pre)

        rospy.signal_shutdown("done")

    # ---------- TF ----------
    def _compute_tip_vector_ee(self):
        try:
            tfm = self.tf_buf.lookup_transform(self.ee_link, self.tip_frame, rospy.Time(0), rospy.Duration(2.0))
            v = np.array([tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z], float)
            rospy.loginfo("[fixedZ] EE->tip from TF: [%.4f, %.4f, %.4f]", v[0], v[1], v[2])
            return v
        except Exception as e:
            rospy.logwarn("[fixedZ] TF EE->tip unavailable (%s). Using tip_to_contact=%.4f along +Z_EE.",
                          str(e), self.tip_to_contact)
            return None

    def _r_tip(self):
        return self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)

    # ---------- MoveIt helpers ----------
    def _execute_joint_sequence(self, joints_seq, dt):
        jt = JointTrajectory()
        jt.joint_names = list(self.joint_names)

        t = 0.0
        for jd in joints_seq:
            pt = JointTrajectoryPoint()
            pt.positions = [jd[n] for n in jt.joint_names]
            t += max(float(dt), 0.05)
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)

        traj = RobotTrajectory()
        traj.joint_trajectory = jt

        if not self.execute:
            rospy.logwarn("[fixedZ] execute=false: skipping trajectory execution.")
            return True

        ok = self.group.execute(traj, wait=True)
        self.group.stop()
        return bool(ok)

    def _go_to_joints(self, joint_vals, tag="pose", fallback_steps=30):
        if not isinstance(joint_vals, (list, tuple)) or len(joint_vals) == 0:
            rospy.loginfo("[fixedZ] Skip %s (no joints provided).", tag)
            return True
        if len(joint_vals) != len(self.joint_names):
            rospy.logerr("[fixedZ] %s joint list size mismatch (%d != %d).", tag, len(joint_vals), len(self.joint_names))
            return False

        target = {n: float(v) for n, v in zip(self.joint_names, joint_vals)}

        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(target)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if ok:
            rospy.loginfo("[fixedZ] Reached %s (planner).", tag)
            return True

        rospy.logwarn("[fixedZ] Planner failed for %s. Falling back to joint interpolation.", tag)
        cur = np.array(self.group.get_current_joint_values(), float)
        tgt = np.array([target[n] for n in self.joint_names], float)

        seq = []
        for s in np.linspace(0.0, 1.0, max(2, int(fallback_steps))):
            q = (1.0 - s) * cur + s * tgt
            seq.append({n: float(v) for n, v in zip(self.joint_names, q)})
        return self._execute_joint_sequence(seq, dt=self.fallback_dt)

    # ---------- IK ----------
    def _ensure_ik(self, timeout=3.0):
        if self.ik_srv is not None:
            return True

        candidates = []
        if self.ik_service_param:
            candidates.append(self.ik_service_param)
        ns = rospy.get_namespace()
        if ns != "/":
            candidates.append(ns + "compute_ik")
        candidates.append("/compute_ik")

        for name in candidates:
            try:
                rospy.wait_for_service(name, timeout=timeout)
                self.ik_srv = rospy.ServiceProxy(name, GetPositionIK)
                rospy.loginfo("[fixedZ] Using IK service: %s", name)
                return True
            except Exception:
                continue

        rospy.logerr("[fixedZ] No IK service available (tried: %s).", ", ".join(candidates))
        return False

    def _ik_solve(self, pose_stamped, avoid=True):
        if not self._ensure_ik(3.0):
            return None

        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(self.ik_timeout)
        req.ik_request.avoid_collisions = bool(avoid)
        req.ik_request.robot_state = self.robot.get_current_state()

        try:
            resp = self.ik_srv(req)
        except rospy.ServiceException:
            return None

        if resp.error_code.val != 1:
            return None

        js = resp.solution.joint_state
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        out = {n: float(name_to_pos[n]) for n in self.joint_names if n in name_to_pos}
        return out if len(out) == len(self.joint_names) else None

    def _ik_for_poses(self, poses):
        ps = PoseStamped()
        ps.header.frame_id = self.ref_frame

        joints_seq = []
        solved = 0

        cur = self.group.get_current_joint_values()
        joints_seq.append({n: float(v) for n, v in zip(self.joint_names, cur)})

        for i, pose in enumerate(poses, start=1):
            ps.header.stamp = rospy.Time.now()
            ps.pose = pose

            tgt = self._ik_solve(ps, True) or self._ik_solve(ps, False)
            if tgt is None:
                frac = solved / float(len(poses))
                if self.allow_partial and frac >= self.min_partial_fraction:
                    rospy.logwarn("[fixedZ] IK failed at %d/%d, accepting partial (frac=%.2f).", i, len(poses), frac)
                    return joints_seq, solved, True
                rospy.logerr("[fixedZ] IK failed at %d/%d (frac=%.2f).", i, len(poses), frac)
                return None, solved, False

            joints_seq.append(tgt)
            solved += 1

        return joints_seq, solved, False

    # ---------- Cloud ----------
    def _read_cloud_xyz(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([p[0], p[1], p[2]])
        P = np.asarray(pts, float)
        return P

    def _wait_cloud(self):
        rospy.loginfo("[fixedZ] Waiting for cloud: %s", self.cloud_topic)
        msg = rospy.wait_for_message(self.cloud_topic, PointCloud2, timeout=self.wait_for_cloud_sec)
        P = self._read_cloud_xyz(msg)
        if P.shape[0] == 0:
            return None, None
        kdt = cKDTree(P) if (cKDTree is not None) else None
        return P, kdt

    # ---------- P0 selection ----------
    def _pick_p0_ahead_of_tip(self, P):
        cur_pose = self.group.get_current_pose(self.ee_link).pose
        q_now = np.array([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        p_ee = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z], float)

        p_tip = p_ee + R_now.dot(self._r_tip())
        dirn = -unit(R_now[:, 2])  # forward ray along -Z_tool

        r = P - p_tip[None, :]
        t = np.einsum("ij,j->i", r, dirn)
        idxs = np.where(t > 0.0)[0]
        if idxs.size == 0:
            idxs = np.arange(P.shape[0])
            t = np.maximum(t, 0.0)

        d2 = np.einsum("ij,ij->i", r, r) - t * t
        best = int(idxs[np.argmin(d2[idxs])])

        p0 = P[best]
        rospy.loginfo("[fixedZ] Picked P0=[%.4f, %.4f, %.4f] idx=%d", p0[0], p0[1], p0[2], best)
        return p0, q_now, p_tip

    # ---------- Fixed-Z orientation ----------
    def _fixed_z_quat(self, q_ref, tip_now, target_point):
        z = unit(self.fixed_z_dir)
        if self.fixed_flip_to_face_target:
            to_tgt = unit(np.asarray(target_point, float) - np.asarray(tip_now, float))
            if np.dot(z, to_tgt) < 0.0:
                z = -z
        q = quat_from_z_min_yaw(q_ref, z)
        R = tft.quaternion_matrix(q)[:3, :3]
        return q, R[:, 2]  # (q_fixed, z_tool_actual)

    # ---------- Touch + sweep + retract ----------
    def _touch_waypoints(self, p_ee_now, q_now, p0, q_fixed, z_vec, R_fixed):
        r_tip = self._r_tip()

        tip_final = p0 - self.contact_margin * z_vec
        ee_final = tip_final - R_fixed.dot(r_tip)

        tip_pre = p0 - (self.contact_margin + self.approach_dist) * z_vec
        tip_far = p0 - (self.contact_margin + self.approach_dist + self.retreat_dist) * z_vec

        ee_pre = tip_pre - R_fixed.dot(r_tip)
        ee_far = tip_far - R_fixed.dot(r_tip)

        far = [pose_from((1.0 - s) * p_ee_now + s * ee_far, slerp(q_now, q_fixed, s))
               for s in np.linspace(0.0, 1.0, max(2, self.far_steps))]
        pre = [pose_from((1.0 - s) * ee_far + s * ee_pre, q_fixed)
               for s in np.linspace(0.0, 1.0, max(2, self.pre_steps))]
        app = [pose_from((1.0 - s) * ee_pre + s * ee_final, q_fixed)
               for s in np.linspace(0.0, 1.0, max(2, self.approach_steps))]
        return far + pre + app

    def _project_sweep_dir(self, z_vec):
        # Project preferred sweep direction onto plane orthogonal to z_vec.
        pref = unit(self.sweep_pref_dir)
        tdir = pref - np.dot(pref, z_vec) * z_vec
        if np.linalg.norm(tdir) < 1e-6:
            # Fallback: pick any vector orthogonal to z_vec.
            a = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(a, z_vec)) > 0.9:
                a = np.array([0.0, 1.0, 0.0])
            tdir = a - np.dot(a, z_vec) * z_vec
        return unit(tdir)

    def _nearest(self, kdt, P, x):
        if kdt is not None:
            _, idx = kdt.query(np.asarray(x, float))
            return int(idx)
        d2 = np.sum((P - np.asarray(x, float)[None, :]) ** 2, axis=1)
        return int(np.argmin(d2))

    def _build_sweep_track(self, P, kdt, p0, z_vec):
        tdir = self._project_sweep_dir(z_vec)
        p_end = np.asarray(p0, float) + self.sweep_length * tdir

        pts = []
        last = None
        for s in np.linspace(0.0, 1.0, max(2, int(self.sweep_samples))):
            pw = (1.0 - s) * np.asarray(p0, float) + s * p_end
            idx = self._nearest(kdt, P, pw)
            pi = P[idx]
            if last is None or np.linalg.norm(pi - last) > 1e-4:
                pts.append(pi)
                last = pi

        if len(pts) < 2:
            return None
        return np.asarray(pts, float)

    def _poses_from_track(self, track_pts, q_fixed, z_vec, R_fixed):
        r_tip = self._r_tip()
        poses = []
        for pi in track_pts:
            tip_target = pi - self.sweep_contact_margin * z_vec
            ee_target = tip_target - R_fixed.dot(r_tip)
            poses.append(pose_from(ee_target, q_fixed))
        return poses

    def _retract_poses(self, current_tip, q_fixed, z_vec, R_fixed, steps=14):
        # Retract along -Z_tool (opposite direction of z_vec).
        lift = self.approach_dist + self.retreat_dist
        r_tip = self._r_tip()

        tip_up = np.asarray(current_tip, float) - lift * z_vec
        ee_up = tip_up - R_fixed.dot(r_tip)

        ee_now_pose = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([ee_now_pose.position.x, ee_now_pose.position.y, ee_now_pose.position.z], float)

        poses = [pose_from((1.0 - s) * p_now + s * ee_up, q_fixed)
                 for s in np.linspace(0.0, 1.0, max(2, int(steps)))]
        return poses

    def _touch_sweep_retract(self):
        P, kdt = self._wait_cloud()
        if P is None:
            rospy.logerr("[fixedZ] Empty or missing cloud.")
            return

        # Current EE pose
        cur_pose = self.group.get_current_pose(self.ee_link).pose
        p_ee_now = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z], float)
        q_now = np.array([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w], float)

        # P0 selection
        p0, _, tip_now = self._pick_p0_ahead_of_tip(P)

        # Fixed-Z orientation
        q_fixed, z_vec = self._fixed_z_quat(q_now, tip_now, p0)
        R_fixed = tft.quaternion_matrix(q_fixed)[:3, :3]

        # Touch
        poses_touch = self._touch_waypoints(p_ee_now, q_now, p0, q_fixed, z_vec, R_fixed)
        joints_touch, solved, _ = self._ik_for_poses(poses_touch)
        if joints_touch is None:
            rospy.logerr("[fixedZ] IK failed for touch.")
            return
        rospy.loginfo("[fixedZ] Touch: solved=%d/%d", solved, len(poses_touch))
        if not self._execute_joint_sequence(joints_touch, dt=self.touch_step_time):
            rospy.logerr("[fixedZ] Touch execution failed.")
            return

        # Sweep
        track = self._build_sweep_track(P, kdt, p0, z_vec)
        if track is None:
            rospy.logerr("[fixedZ] Sweep track generation failed.")
            return
        poses_sweep = self._poses_from_track(track, q_fixed, z_vec, R_fixed)
        joints_sweep, solved2, _ = self._ik_for_poses(poses_sweep)
        if joints_sweep is None:
            rospy.logerr("[fixedZ] IK failed for sweep.")
            return
        rospy.loginfo("[fixedZ] Sweep: solved=%d/%d", solved2, len(poses_sweep))
        if not self._execute_joint_sequence(joints_sweep, dt=self.sweep_step_time):
            rospy.logerr("[fixedZ] Sweep execution failed.")
            return

        # Retract
        tip_last = track[-1] - self.sweep_contact_margin * z_vec
        poses_up = self._retract_poses(tip_last, q_fixed, z_vec, R_fixed, steps=14)
        joints_up, solved3, _ = self._ik_for_poses(poses_up)
        if joints_up is None:
            rospy.logwarn("[fixedZ] IK failed for retract. Skipping retract.")
            return
        rospy.loginfo("[fixedZ] Retract: solved=%d/%d", solved3, len(poses_up))
        self._execute_joint_sequence(joints_up, dt=self.touch_step_time)


def main():
    try:
        TouchAndSweepFixedZ()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()


if __name__ == "__main__":
    main()
