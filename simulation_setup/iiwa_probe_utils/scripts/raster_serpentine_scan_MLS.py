#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
raster_serpentine_scan.py

Touch → raster sweep → retract with FIXED-Z tool orientation.

Two raster styles:
  1) raster_style="parallel": serpentine made of lanes PARALLEL to the centerline
  2) raster_style="cross"   : serpentine made of TRANSVERSE lines perpendicular to the centerline

Main parameters:
  ~sweep_length          : centerline length P0→PF (m)
  ~raster_width          : length of each scan line (m)
  ~raster_line_spacing   : spacing between consecutive lines along the centerline (m)
  ~samples_per_line      : number of samples along each line
  ~raster_bridge_samples : number of bridging samples between lines (to smooth line transitions)

Key improvement (anti-jagged trajectory):
- Surface projection via local MLS/PCA plane fitting (k-neighborhood) instead of pure nearest-point snapping.
- Light smoothing + optional re-projection to keep the tip on the surface.

MLS parameters:
  ~mls_enable (bool)   : enable MLS (default True)
  ~mls_k (int)         : PCA neighbors (default 25)
  ~mls_alpha (float)   : smoothing factor 0..1 (default 0.35). Lower → smoother (more filtering)
  ~mls_reproject (bool): re-project after smoothing (default True)

"""

import sys
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  
import tf.transformations as tft
import ast

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from moveit_commander import (
    roscpp_initialize,
    roscpp_shutdown,
    MoveGroupCommander,
    RobotCommander
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Optional fast nearest-neighbor structure (falls back to brute force if missing)
try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


def norm(v, eps=1e-12):
    """Return a unit vector (safe normalization)."""
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def quat_from_axes(x, y, z):
    """
    Build a quaternion from an orthonormal basis (x,y,z as columns of a rotation matrix).
    """
    R = np.column_stack((norm(x), norm(y), norm(z)))
    T = np.eye(4)
    T[:3, :3] = R
    qx, qy, qz, qw = tft.quaternion_from_matrix(T)
    return np.array([qx, qy, qz, qw], float)


def quat_from_z_min_yaw(q_prev, z_des):
    """
    Compute a quaternion whose Z axis matches z_des, while keeping the yaw as close as possible
    to the previous orientation q_prev (i.e., minimal rotation around the Z axis).
    """
    q_prev = np.asarray(q_prev, float)
    Rprev = tft.quaternion_matrix(q_prev)[:3, :3]
    x_prev = Rprev[:, 0]

    z = norm(z_des)
    # Project previous X onto the plane orthogonal to z to preserve yaw as much as possible
    x_proj = x_prev - np.dot(x_prev, z) * z

    # If projection degenerates, choose an arbitrary "up" and project it
    if np.linalg.norm(x_proj) < 1e-6:
        up = np.array([0, 0, 1.0])
        x_proj = up - np.dot(up, z) * z
        if np.linalg.norm(x_proj) < 1e-6:
            up = np.array([1, 0, 0.0])
            x_proj = up - np.dot(up, z) * z

    x = norm(x_proj)
    y = norm(np.cross(z, x))
    x = norm(np.cross(y, z))
    return quat_from_axes(x, y, z)
def parse_vec3_param(name, default_xyz):
    # Accept list/tuple or a string like "[0,1,0]" or "0 1 0".
    v = rospy.get_param(name, default_xyz)
    if isinstance(v, (list, tuple)) and len(v) == 3:
        return np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float)
    if isinstance(v, str):
        s = v.strip()
        try:
            x = ast.literal_eval(s)
            if isinstance(x, (list, tuple)) and len(x) == 3:
                return np.array([float(x[0]), float(x[1]), float(x[2])], dtype=float)
        except Exception:
            pass
        parts = s.replace(",", " ").split()
        if len(parts) == 3:
            return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)
    return np.array(default_xyz, dtype=float)


def slerp(q0, q1, s):
    """Quaternion SLERP interpolation helper."""
    return np.array(tft.quaternion_slerp(q0, q1, s), float)


def pose_from(p, q):
    """Create geometry_msgs/Pose from position p (3,) and quaternion q (4,)."""
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = float(p[0]), float(p[1]), float(p[2])
    ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q.tolist()
    return ps


def to_dict(names, vals):
    """Utility: map joint names to joint values (for JointTrajectoryPoint building)."""
    return {n: float(v) for n, v in zip(names, vals)}


class PreToPoseAndTouchFixedZ(object):
    """
    Main pipeline:
      1) Move robot to pre-approach joints
      2) Move robot to a target configuration near the patient
      3) Select a contact point P0 on the surface in front of the probe tip
      4) Perform a fixed-Z 'touch' approach until contact margin is reached
      5) Generate a serpentine raster track on the surface (parallel/cross)
      6) Convert surface track to end-effector poses while keeping fixed-Z orientation
      7) Solve IK for all poses and execute the joint trajectory
      8) Retract and return to pre-approach configuration
    """
    def __init__(self):
        # Initialize MoveIt C++ bindings and ROS node
        roscpp_initialize(sys.argv)
        rospy.init_node("raster_serpentine_scan_fixedZ")

        # ---------------- MoveIt / planning group configuration ----------------
        self.group_name = rospy.get_param("~group_name", "manipulator")
        self.ee_link = rospy.get_param("~ee_link", "iiwa_link_ee")
        self.ref_frame = rospy.get_param("~ref_frame", "world")
        self.speed_scale = float(rospy.get_param("~speed_scale", 0.20))

        # ---------------- Key robot configurations (joint-space) ----------------
        # pre_joints: safe configuration (away from the patient)
        self.pre_joints = rospy.get_param(
            "~pre_joints",
            [-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]
        )
        # target_joints: configuration close to the scanning region
        self.target_joints = rospy.get_param(
            "~target_joints",
            [0.051, 0.849, 2.083, 0.542, 0.821, -1.966, 2.934]
        )
        # Fallback settings (used when planner fails; executes a direct joint interpolation)
        self.fallback_steps_pre = int(rospy.get_param("~fallback_steps_pre", 30))
        self.fallback_steps_tgt = int(rospy.get_param("~fallback_steps_tgt", 40))
        self.fallback_dt = float(rospy.get_param("~fallback_dt", 0.20))

        # ---------------- Cloud + touch approach parameters ----------------
        self.cloud_topic = rospy.get_param("~cloud_topic", "/skin_cloud")
        self.contact_margin = float(rospy.get_param("~contact_margin", 0.006))   # tip stays slightly "inside" along z_vec
        self.approach_dist = float(rospy.get_param("~approach_dist", 0.06))      # pre-contact distance
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 0.10))        # post-scan retract distance

        # Number of samples used for the approach segments and execution timing
        self.far_steps = int(rospy.get_param("~far_steps", 25))
        self.pre_steps = int(rospy.get_param("~pre_steps", 18))
        self.approach_steps = int(rospy.get_param("~approach_steps", 22))
        self.step_time = float(rospy.get_param("~step_time", 0.25))

        # ---------------- IK configuration ----------------
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 1.2))
        self.ik_service_param = rospy.get_param("~ik_service", "")
        self.allow_partial = bool(rospy.get_param("~allow_partial", True))
        self.min_partial_fraction = float(rospy.get_param("~min_partial_fraction", 0.20))

        # ---------------- Tip definition ----------------
        # tip_frame is the probe tip frame; we use TF to compute EE->tip vector
        self.tip_frame = rospy.get_param("~tip_frame", "probe_tip")
        self.tip_to_contact = float(rospy.get_param("~tip_to_contact", 0.0))  # fallback if TF is unavailable
        self.tip_vec_ee = None

        # ---------------- Sweep / raster parameters ----------------
        self.sweep_length = float(rospy.get_param("~sweep_length", 0.12))
        self.sweep_step_time = float(rospy.get_param("~sweep_step_time", 0.20))
        self.sweep_contact_margin = float(rospy.get_param("~sweep_contact_margin", self.contact_margin))
        # Preferred direction used to define tangents on the surface (helps remove sign ambiguity)
        self.sweep_pref_dir = parse_vec3_param("~sweep_pref_dir", [0.0, 1.0, 0.0])
        # Raster layout
        self.raster_enable = bool(rospy.get_param("~raster_enable", True))
        self.raster_style = rospy.get_param("~raster_style", "cross").strip().lower()  # "cross" or "parallel"
        self.raster_width = float(rospy.get_param("~raster_width", 0.06))
        self.raster_line_spacing = float(rospy.get_param("~raster_line_spacing", 0.006))
        self.samples_per_line = int(rospy.get_param("~samples_per_line", 40))
        self.raster_bridge_samples = int(rospy.get_param("~raster_bridge_samples", 10))

        # ---------------- Fixed-Z constraint ----------------
        # fixed_z_dir is the desired tool Z axis in the world frame (e.g., [0,0,-1])
        self.fixed_z_dir    = parse_vec3_param("~fixed_z_dir",    [0.0, 0.0, -1.0])
        # If True, flip fixed_z_dir so that it points toward the target point
        self.fixed_flip_to_face_target = bool(rospy.get_param("~fixed_flip_to_face_target", True))

        # ---------------- Robust cloud parsing ----------------
        self.force_ignore_normals = bool(rospy.get_param("~force_ignore_normals", False))

        # ---------------- MLS smoothing/projection (NEW) ----------------
        # Local plane fit via PCA + smoothing to reduce jaggedness due to sparse/discrete clouds
        self.mls_enable = bool(rospy.get_param("~mls_enable", True))
        self.mls_k = int(rospy.get_param("~mls_k", 25))
        self.mls_alpha = float(rospy.get_param("~mls_alpha", 0.35))     # 0..1 blend factor
        self.mls_reproject = bool(rospy.get_param("~mls_reproject", True))
        self.mls_alpha = float(np.clip(self.mls_alpha, 0.0, 1.0))
        if self.mls_k < 5:
            self.mls_k = 5

        # ---------------- MoveIt setup ----------------
        # In a namespaced robot (e.g. ROS_NAMESPACE=iiwa), robot_description lives under that namespace.
        ns = rospy.get_namespace()
        robot_description = (ns if ns != "/" else "") + "robot_description"
        self.robot = RobotCommander(robot_description=robot_description, ns=ns)
        self.group = MoveGroupCommander(self.group_name, robot_description=robot_description, ns=ns)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_pose_reference_frame(self.ref_frame)
        self.group.set_max_velocity_scaling_factor(self.speed_scale)
        self.group.set_max_acceleration_scaling_factor(self.speed_scale)
        self.joint_names = self.group.get_active_joints()

        # ---------------- TF + point cloud subscription ----------------
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf)

        self.points = None     # Nx3 cloud points
        self.normals = None    # Nx3 optional normals (if present and valid)
        self.kdt = None        # KD-tree for fast neighbor queries
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        # IK service proxy (resolved at first use)
        self.ik_srv = None
        self.ik_name = None

        # Compute EE->tip vector using TF
        self.compute_tip_vector()

        rospy.loginfo("MLS: enable=%s k=%d alpha=%.2f reproject=%s",
                      str(self.mls_enable), int(self.mls_k), float(self.mls_alpha), str(self.mls_reproject))

        # ---------------- Execute full pipeline ----------------
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        self.step_to_joints(self.target_joints, self.fallback_steps_tgt, "target_pose")
        self.touch_and_sweep_fixedZ()
        rospy.signal_shutdown("done")

    # ---------------- Cloud callback (robust parsing) ----------------
    def cloud_cb(self, msg):
        """
        Parse PointCloud2 into numpy arrays.
        - If normals are available (normal_x/y/z or nx/ny/nz), store them.
        - Build KD-tree if SciPy is available for fast nearest neighbor queries.
        """
        try:
            fields = [f.name for f in msg.fields]
            has_norm = (("normal_x" in fields or "nx" in fields) and
                        ("normal_y" in fields or "ny" in fields) and
                        ("normal_z" in fields or "nz" in fields))

            # Optionally ignore normals even if present (forces PCA normals later)
            if self.force_ignore_normals:
                has_norm = False

            # Detect actual normal field names
            nx = "normal_x" if "normal_x" in fields else ("nx" if "nx" in fields else None)
            ny = "normal_y" if "normal_y" in fields else ("ny" if "ny" in fields else None)
            nz = "normal_z" if "normal_z" in fields else ("nz" if "nz" in fields else None)

            pts = []
            nors = [] if has_norm else None

            # Read points (and normals if available)
            if has_norm and nx and ny and nz:
                for p in pc2.read_points(msg, field_names=("x", "y", "z", nx, ny, nz), skip_nans=True):
                    pts.append([p[0], p[1], p[2]])
                    nors.append([p[3], p[4], p[5]])
            else:
                for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    pts.append([p[0], p[1], p[2]])

            # Handle empty cloud
            if len(pts) == 0:
                self.points = None
                self.normals = None
                self.kdt = None
                return

            self.points = np.asarray(pts, dtype=np.float64)

            # Normalize normals if valid
            self.normals = None
            if has_norm and nors is not None and len(nors) == len(pts):
                n_arr = np.asarray(nors, dtype=np.float64)
                if n_arr.ndim == 2 and n_arr.shape[1] == 3:
                    nrm = np.linalg.norm(n_arr, axis=1)
                    nrm[~np.isfinite(nrm)] = 0.0
                    nrm[nrm < 1e-12] = 1.0
                    self.normals = n_arr / nrm[:, None]
                else:
                    rospy.logwarn_throttle(2.0, "Normals present but invalid format: ignoring normals and using PCA.")

            # Build KD-tree (optional)
            if cKDTree is not None:
                try:
                    self.kdt = cKDTree(self.points)
                except Exception:
                    self.kdt = None
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cloud_cb parsing error (%s). Ignoring message.", str(e))
            self.points = None
            self.normals = None
            self.kdt = None

    # Tip vector 
    def compute_tip_vector(self):
        """
        Compute the vector from end-effector link to probe tip frame using TF.
        This allows converting between EE pose and actual probe tip position.
        """
        try:
            tfm = self.tf_buf.lookup_transform(self.ee_link, self.tip_frame,
                                               rospy.Time(0), rospy.Duration(2.0))
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y 
            tz = tfm.transform.translation.z
            self.tip_vec_ee = np.array([tx, ty, tz], float)
            rospy.loginfo("tip_frame=%s -> EE->tip = [%.3f, %.3f, %.3f] in %s",
                          self.tip_frame, tx, ty, tz, self.ee_link)
        except Exception as e:
            # If TF is missing, we can still proceed using a scalar offset along EE Z
            self.tip_vec_ee = None
            rospy.logwarn("lookup_transform(%s->%s) failed: %s. Fallback tip_to_contact=%.3f",
                          self.ee_link, self.tip_frame, str(e), self.tip_to_contact)

    # Joint helpers 
    def exec_joint_traj(self, joints_seq, dt=None):
        """
        Execute a joint trajectory built from a list of joint dictionaries.
        dt controls the time step between points (simple time-parameterization).
        """
        if not joints_seq or len(joints_seq) < 2:
            return False

        jt = JointTrajectory()
        jt.joint_names = list(self.joint_names)
        t = 0.0
        step = self.fallback_dt if dt is None else dt

        for jd in joints_seq:
            pt = JointTrajectoryPoint()
            pt.positions = [jd[n] for n in jt.joint_names]
            pt.velocities = [0.0] * len(jt.joint_names)
            pt.accelerations = [0.0] * len(jt.joint_names)
            t += max(step, 0.05)
            pt.time_from_start = rospy.Duration(t)
            jt.points.append(pt)

        traj = RobotTrajectory()
        traj.joint_trajectory = jt

        ok = self.group.execute(traj, wait=True)
        self.group.stop()
        return bool(ok)

    def step_to_joints(self, joint_vals, steps, tag):
        """
        Move to a joint configuration.
        - First tries MoveIt planning (group.go()).
        - If it fails, falls back to direct linear interpolation in joint space.
        """
        jd = {}
        for i in range(7):
            name = f"iiwa_joint_{i+1}"
            if name in self.joint_names:
                jd[name] = float(joint_vals[i])

        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(jd)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if ok:
            rospy.loginfo("Reached %s via planner.", tag)
            return True

        # Fallback: simple interpolation if planner fails
        cur = np.array(self.group.get_current_joint_values(), float)
        tgt = np.array([jd[n] for n in self.joint_names], float)
        seq = []
        for s in np.linspace(0.0, 1.0, max(2, int(steps))):
            pos = (1.0 - s) * cur + s * tgt
            seq.append({n: float(v) for n, v in zip(self.joint_names, pos)})

        rospy.logwarn("Planner failed for %s. Using fallback interpolation (%d steps).", tag, len(seq))
        return self.exec_joint_traj(seq, self.fallback_dt)

    # ---------------- IK ----------------
    def ensure_ik(self, timeout=3.0):
        """
        Resolve and connect to a MoveIt IK service.
        Tries (in order): user-provided ~ik_service, namespaced compute_ik, global /compute_ik.
        """
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
                self.ik_name = name
                rospy.loginfo("Using IK service: %s", name)
                return True
            except Exception:
                pass

        rospy.logerr("No IK service available among: %s", ", ".join(candidates))
        return False

    def ik_solve(self, pose_stamped, avoid=True):
        """
        Solve IK for a single PoseStamped.
        Returns a dict {joint_name: value} if successful, otherwise None.
        """
        if not self.ensure_ik(3.0):
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
        except rospy.ServiceException as e:
            rospy.logwarn("IK service failed: %s", str(e))
            return None

        if not hasattr(resp, "error_code") or resp.error_code.val != 1:
            return None

        js = resp.solution.joint_state
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        target = {n: name_to_pos[n] for n in self.joint_names if n in name_to_pos}
        return target if len(target) == len(self.joint_names) else None

    def ik_for_all(self, poses, frame_id):
        """
        Solve IK for a sequence of end-effector poses.
        If allow_partial is True, can return a partial prefix solution if enough waypoints are solved.
        """
        ps = PoseStamped()
        ps.header.frame_id = frame_id

        joints_seq = []
        solved = 0

        # Start trajectory from current joint state
        cur = self.group.get_current_joint_values()
        joints_seq.append(to_dict(self.joint_names, cur))

        for i, pose in enumerate(poses, 1):
            ps.header.stamp = rospy.Time.now()
            ps.pose = pose

            # Try IK with collision avoidance; if it fails, retry without
            tgt = self.ik_solve(ps, True) or self.ik_solve(ps, False)
            if tgt is None:
                frac = solved / float(len(poses))
                if self.allow_partial and frac >= self.min_partial_fraction:
                    rospy.logwarn("IK failed at waypoint %d/%d: accepting partial (frac=%.2f).",
                                  i, len(poses), frac)
                    return joints_seq, solved, True
                rospy.logerr("IK failed at waypoint %d/%d (frac=%.2f).", i, len(poses), frac)
                return None, solved, False

            joints_seq.append(tgt)
            solved += 1

        return joints_seq, solved, False

    # ---------------- Geometry helpers ----------------
    def nearest_index(self, p):
        """
        Return index of nearest cloud point to p.
        Uses KD-tree if available, otherwise brute force.
        """
        if self.points is None or len(self.points) == 0:
            return None

        p = np.asarray(p, float)
        if self.kdt is not None:
            _, idx = self.kdt.query(p)
            return int(idx)

        d2 = np.sum((self.points - p[None, :]) ** 2, axis=1)
        return int(np.argmin(d2))

    def normal_at(self, idx_or_point, k=25):
        """
        Estimate the surface normal either:
          - directly from stored normals (if available), or
          - via PCA on k neighbors (fallback).
        """
        if self.points is None or len(self.points) == 0:
            return np.array([0, 0, 1.0], float)

        if isinstance(idx_or_point, int):
            if self.normals is not None and idx_or_point < len(self.normals):
                return norm(self.normals[idx_or_point])
            p = self.points[idx_or_point]
        else:
            p = np.asarray(idx_or_point, float)

        # Gather neighborhood
        if self.kdt is not None:
            k_ = min(k, len(self.points))
            _, idxs = self.kdt.query(p, k=k_)
            neigh = self.points[idxs if k_ > 1 else [idxs]]
        else:
            d2 = np.sum((self.points - p[None, :]) ** 2, axis=1)
            idxs = np.argsort(d2)[:min(k, len(self.points))]
            neigh = self.points[idxs]

        # PCA: normal is the smallest-variance direction
        c = neigh.mean(axis=0)
        A = neigh - c
        try:
            _, _, vh = np.linalg.svd(A, full_matrices=False)
            n = vh[-1, :]
        except Exception:
            n = np.array([0, 0, 1.0], float)

        return norm(n)

    def tangent_from_normal(self, n, pref=None):
        """
        Compute a stable tangent direction given a normal and a preferred direction.
        The preferred direction is projected onto the tangent plane and normalized.
        """
        prefv = np.array(pref if pref is not None else [1.0, 0.0, 0.0], float)
        n = norm(n)
        t = prefv - np.dot(prefv, n) * n

        # If the projection is degenerate, try alternative axes
        if np.linalg.norm(t) < 1e-6:
            pref2 = np.array([0.0, 1.0, 0.0], float)
            t = pref2 - np.dot(pref2, n) * n
            if np.linalg.norm(t) < 1e-6:
                pref2 = np.array([0.0, 0.0, 1.0], float)
                t = pref2 - np.dot(pref2, n) * n

        return norm(t)

    # ---------------- MLS projection (NEW) ----------------
    def project_mls(self, Pw, k=None):
        """
        Project a query point Pw onto a locally fitted plane (PCA over k neighbors).
        Returns:
          - Pproj: Pw projected onto local plane
          - n_plane: estimated plane normal
        """
        if self.points is None or len(self.points) == 0:
            return None, None

        Pw = np.asarray(Pw, float)
        k_use = int(self.mls_k if k is None else k)
        k_use = int(min(max(5, k_use), len(self.points)))

        # Gather neighbors
        if self.kdt is not None:
            _, idxs = self.kdt.query(Pw, k=k_use)
            idxs = np.atleast_1d(idxs)
            neigh = self.points[idxs]
        else:
            d2 = np.sum((self.points - Pw[None, :]) ** 2, axis=1)
            idxs = np.argsort(d2)[:k_use]
            neigh = self.points[idxs]

        # PCA plane: centroid + normal from smallest singular vector
        c = neigh.mean(axis=0)
        A = neigh - c
        try:
            _, _, vh = np.linalg.svd(A, full_matrices=False)
            n = vh[-1, :]
        except Exception:
            n = np.array([0.0, 0.0, 1.0], float)

        n = norm(n)
        Pproj = Pw - np.dot(Pw - c, n) * n
        return Pproj, n

    def project_to_surface(self, Pw):
        """
        Surface projection wrapper:
          - if MLS enabled: project via local plane fit
          - else: snap to nearest cloud point
        """
        if self.mls_enable:
            Pproj, _ = self.project_mls(Pw, k=self.mls_k)
            if Pproj is not None:
                return Pproj

        # Fallback: nearest point in the cloud
        idx = self.nearest_index(Pw)
        if idx is None:
            return None
        return self.points[idx]

    def addP_surface(self, track, Pw):
        """
        Append a point to a surface track while reducing jaggedness:
          1) project candidate point Pw onto surface
          2) optionally smooth it with exponential-like blending to previous point
          3) optionally re-project to remain on the surface
          4) append if sufficiently far from last point
        """
        P1 = self.project_to_surface(Pw)
        if P1 is None:
            return

        # Blend with previous point to reduce high-frequency jitter
        if len(track) > 0 and self.mls_alpha > 0.0:
            P1 = self.mls_alpha * P1 + (1.0 - self.mls_alpha) * track[-1]

            # Re-project after smoothing to keep contact on surface
            if self.mls_enable and self.mls_reproject:
                P1 = self.project_to_surface(P1)
                if P1 is None:
                    return

        if len(track) == 0 or np.linalg.norm(P1 - track[-1]) > 1e-4:
            track.append(P1)

    # ---------------- Fixed Z orientation ----------------
    def z_fixed_facing_target(self, q_ref, tip_pos_now, target_point):
        """
        Compute the fixed Z axis to use for scanning and a quaternion that enforces it.
        Optionally flips Z to face the target point.
        """
        zfix = norm(self.fixed_z_dir)
        if self.fixed_flip_to_face_target:
            to_tgt = norm(np.asarray(target_point, float) - np.asarray(tip_pos_now, float))
            if np.dot(zfix, to_tgt) < 0.0:
                zfix = -zfix

        q_des = quat_from_z_min_yaw(q_ref, zfix)
        return zfix, q_des

    # ---------------- Touch waypoints ----------------
    def make_waypoints_fixedZ(self, p_now, q_now, P0):
        """
        Create an approach trajectory (poses) to bring the probe tip into contact with P0,
        moving along the fixed-Z direction and keeping a contact margin.
        """
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        z_vec, q_des = self.z_fixed_facing_target(q_now, p_tip_now, P0)
        R_des = tft.quaternion_matrix(q_des)[:3, :3]

        # Tip target in contact and corresponding EE pose
        tip_final = P0 - self.contact_margin * z_vec
        ee_final = tip_final - R_des.dot(r_tip)

        # Pre-contact and far poses along z_vec
        tip_pre = P0 - (self.contact_margin + self.approach_dist) * z_vec
        tip_far = P0 - (self.contact_margin + self.approach_dist + self.retreat_dist) * z_vec
        ee_pre = tip_pre - R_des.dot(r_tip)
        ee_far = tip_far - R_des.dot(r_tip)

        # Interpolate poses:
        # - far segment: translate + rotate gradually
        # - pre + approach: translate with fixed final orientation
        way_far = [pose_from((1.0 - s) * p_now + s * ee_far, slerp(q_now, q_des, s))
                   for s in np.linspace(0.0, 1.0, max(2, self.far_steps))]
        way_pre = [pose_from((1.0 - s) * ee_far + s * ee_pre, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.pre_steps))]
        way_app = [pose_from((1.0 - s) * ee_pre + s * ee_final, q_des)
                   for s in np.linspace(0.0, 1.0, max(2, self.approach_steps))]

        return way_far + way_pre + way_app, tip_final

    # ---------------- P0 selection ----------------
    def pick_P0_ahead_of_tip(self):
        """
        Select an initial contact point P0 on the surface:
        - waits until a point cloud is available
        - computes current probe tip position
        - searches points in front of the tip along -Z of the current tool pose
        - picks the closest point to the ray (min distance to line)
        """
        for _ in range(200):
            if self.points is not None and len(self.points) > 0:
                break
            rospy.sleep(0.05)

        if self.points is None or len(self.points) == 0:
            rospy.logerr("No cloud available on %s", self.cloud_topic)
            return None, None

        cur = self.group.get_current_pose(self.ee_link).pose
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip = np.array([cur.position.x, cur.position.y, cur.position.z], float) + R_now.dot(r_tip)

        # "Forward" direction toward the surface: -Z of current EE frame
        z_now = R_now[:, 2]
        dirn = -norm(z_now)

        # Project each point onto the ray and compute distance to ray
        r = self.points - p_tip[None, :]
        t = np.einsum('ij,j->i', r, dirn)
        idxs = np.where(t > 0.0)[0]
        if idxs.size == 0:
            idxs = np.arange(len(self.points))
            t = np.maximum(t, 0.0)

        d2 = np.einsum('ij,ij->i', r, r) - t * t
        best = idxs[np.argmin(d2[idxs])] if idxs.size > 0 else int(np.argmin(d2))

        P0 = self.points[best]
        n0 = self.normals[best] if (self.normals is not None and best < len(self.normals)) else None
        rospy.loginfo("Selected P0: [%.3f, %.3f, %.3f] (idx %d)", P0[0], P0[1], P0[2], best)
        return P0, n0

    # ======================================================================
    #  RASTER STYLE 1: PARALLEL (lanes parallel to centerline)
    # ======================================================================
    def build_centerline_on_surface(self, P0, n0, n_samples):
        """
        Build a centerline track on the surface by sampling along a desired tangential direction
        and projecting each sample onto the surface.
        """
        P0 = np.asarray(P0, float)
        idx0 = self.nearest_index(P0)
        if idx0 is None:
            return None, None

        n = n0 if n0 is not None else self.normal_at(idx0)
        t1 = self.tangent_from_normal(n, self.sweep_pref_dir)
        PF_wish = P0 + self.sweep_length * t1

        pts = []
        nors = []
        last = None
        for s in np.linspace(0.0, 1.0, max(2, int(n_samples))):
            Pw = (1.0 - s) * P0 + s * PF_wish
            Pi = self.project_to_surface(Pw)
            if Pi is None:
                continue

            if last is None or np.linalg.norm(Pi - last) > 1e-4:
                pts.append(Pi)
                # normal: compute from neighborhood around Pi (consistent even when MLS is on)
                idx = self.nearest_index(Pi)
                nors.append(self.normal_at(idx) if idx is not None else np.array([0, 0, 1.0]))
                last = Pi

        if len(pts) < 2:
            return None, None
        return np.asarray(pts, float), np.asarray(nors, float)

    def plan_serpentine_parallel(self, P0, n0):
        """
        Create a boustrophedon path by offsetting the centerline sideways, producing parallel lanes.
        """
        spacing = max(1e-6, float(self.raster_line_spacing))
        width = max(0.0, float(self.raster_width))
        half = 0.5 * width
        if width < 1e-6:
            return None

        center_pts, center_ns = self.build_centerline_on_surface(P0, n0, n_samples=self.samples_per_line)
        if center_pts is None:
            return None

        # Compute lateral direction t2(k) at each centerline sample (t2 = n × t1)
        t2_list = []
        t2_prev = None
        for k in range(len(center_pts)):
            if k < len(center_pts) - 1:
                t1k = norm(center_pts[k + 1] - center_pts[k])
            else:
                t1k = norm(center_pts[k] - center_pts[k - 1])
            nk = norm(center_ns[k])
            t2k = norm(np.cross(nk, t1k))

            # Stabilize sign to avoid left/right flipping
            if t2_prev is not None and np.dot(t2k, t2_prev) < 0.0:
                t2k = -t2k
            t2_prev = t2k
            t2_list.append(t2k)
        t2_list = np.asarray(t2_list, float)

        # Offsets around the centerline: 0, +spacing, -spacing, +2spacing, ...
        n_side = int(np.floor(half / spacing))
        offsets = [0.0]
        for i in range(1, n_side + 1):
            offsets += [+i * spacing, -i * spacing]

        # Build each lane by sampling the offset centerline and projecting to surface
        lanes = []
        for off in offsets:
            lane = []
            last = None
            for Pc, t2k in zip(center_pts, t2_list):
                Pw = Pc + off * t2k
                Pi = self.project_to_surface(Pw)
                if Pi is None:
                    continue
                if last is None or np.linalg.norm(Pi - last) > 1e-4:
                    lane.append(Pi)
                    last = Pi
            if len(lane) >= 2:
                lanes.append(np.asarray(lane, float))

        if not lanes:
            return None

        # Concatenate lanes in serpentine order (reverse every other lane)
        track = []
        bridgeS = max(0, int(self.raster_bridge_samples))

        for li, lane in enumerate(lanes):
            pts_lane = lane if (li % 2) == 0 else lane[::-1]

            for P in pts_lane:
                self.addP_surface(track, P)

            # Optional bridging points between the end of one lane and the start of the next
            if li < len(lanes) - 1 and bridgeS > 0:
                nxt = lanes[li + 1]
                pts_next = nxt if ((li + 1) % 2) == 0 else nxt[::-1]
                P_end = pts_lane[-1]
                P_next = pts_next[0]
                for s in np.linspace(0.0, 1.0, bridgeS + 2)[1:-1]:
                    Pw = (1.0 - s) * P_end + s * P_next
                    self.addP_surface(track, Pw)

        return np.asarray(track, float) if len(track) >= 2 else None

    # ======================================================================
    #  RASTER STYLE 2: CROSS (transverse lines, as in the sketch)
    # ======================================================================
    def plan_serpentine_cross(self, P0, n0):
        """
        Build a cross raster:
          - Generate a centerline of length sweep_length on the surface.
          - At stations spaced by raster_line_spacing, generate a transverse scan line of length raster_width.
          - Concatenate lines in boustrophedon order (reverse every other line).
          - Use bridging points (optionally curved) to smooth transitions between lines.
        """
        row_spacing = max(1e-6, float(self.raster_line_spacing))
        line_len = max(0.0, float(self.raster_width))
        half = 0.5 * line_len
        if line_len < 1e-6:
            return None

        # Number of rows along the centerline
        n_rows = int(np.floor(max(0.0, self.sweep_length) / row_spacing)) + 1
        n_rows = max(2, n_rows)

        # Centerline stations on the surface
        center_pts, center_ns = self.build_centerline_on_surface(P0, n0, n_samples=n_rows)
        if center_pts is None or len(center_pts) < 2:
            return None

        # Transverse direction at each station: t_cross = n × t_centerline
        t_cross_list = []
        tprev = None
        for k in range(len(center_pts)):
            if k < len(center_pts) - 1:
                tcl = norm(center_pts[k + 1] - center_pts[k])
            else:
                tcl = norm(center_pts[k] - center_pts[k - 1])
            nk = norm(center_ns[k])
            tc = norm(np.cross(nk, tcl))

            # Stabilize sign to avoid left/right flips
            if tprev is not None and np.dot(tc, tprev) < 0.0:
                tc = -tc
            tprev = tc
            t_cross_list.append(tc)
        t_cross_list = np.asarray(t_cross_list, float)

        nS = max(2, int(self.samples_per_line))
        bridgeS = max(0, int(self.raster_bridge_samples))

        track = []

        def make_row(Ck, tc, reverse=False):
            """
            Generate one transverse line around station Ck using direction tc.
            Points are projected to surface and locally smoothed.
            """
            us = np.linspace(-half, +half, nS)
            if reverse:
                us = us[::-1]

            row_pts = []
            last = None
            for u in us:
                Pw = Ck + u * tc
                Pi = self.project_to_surface(Pw)
                if Pi is None:
                    continue

                # Optional smoothing inside the row to reduce jitter
                if last is not None and self.mls_alpha > 0.0:
                    Pi2 = self.mls_alpha * Pi + (1.0 - self.mls_alpha) * last
                    if self.mls_enable and self.mls_reproject:
                        Pi2 = self.project_to_surface(Pi2)
                    Pi = Pi2 if Pi2 is not None else Pi

                if last is None or np.linalg.norm(Pi - last) > 1e-4:
                    row_pts.append(Pi)
                    last = Pi

            return np.asarray(row_pts, float) if len(row_pts) >= 2 else None

        # Build all rows
        rows = []
        for k in range(len(center_pts)):
            Ck = center_pts[k]
            tc = t_cross_list[k]
            rev = (k % 2) == 1
            row = make_row(Ck, tc, reverse=rev)
            if row is not None:
                rows.append(row)

        if not rows:
            return None

        # Concatenate rows + smooth transitions between them (quadratic Bezier bridge)
        for k in range(len(rows)):
            row = rows[k]
            for P in row:
                self.addP_surface(track, P)

            if k < len(rows) - 1 and bridgeS > 0:
                P_end = row[-1]
                P_next = rows[k + 1][0]

                # Exit direction from the current row
                v_out = norm(row[-1] - row[-2]) if len(row) >= 2 else norm(P_next - P_end)

                # Bezier control point (push forward to round the corner)
                alpha = float(rospy.get_param("~bridge_alpha", 2.0 * self.raster_line_spacing))
                C = P_end + alpha * v_out

                for s in np.linspace(0.0, 1.0, bridgeS + 2)[1:-1]:
                    # Quadratic Bezier curve between rows
                    Pw = (1 - s) ** 2 * P_end + 2 * (1 - s) * s * C + s ** 2 * P_next
                    self.addP_surface(track, Pw)

        return np.asarray(track, float) if len(track) >= 2 else None

    # ---------------- Surface track → poses (fixed-Z) ----------------
    def poses_from_surface_track_fixedZ(self, q_start, track_pts):
        """
        Convert a surface track (tip positions) into end-effector poses:
          - compute a fixed orientation (fixed-Z constraint)
          - for each surface point Pi, position the tip at contact margin along z_vec
          - convert tip target into EE target using EE->tip vector
        """
        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)
        R_now = tft.quaternion_matrix(q_now)[:3, :3]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)
        p_tip_now = p_now + R_now.dot(r_tip)

        # Determine fixed tool orientation based on fixed_z_dir and current orientation
        z_vec, q_fixed = self.z_fixed_facing_target(q_start, p_tip_now, track_pts[0])
        R_fixed = tft.quaternion_matrix(q_fixed)[:3, :3]

        poses = []
        for Pi in track_pts:
            tip_target = Pi - self.sweep_contact_margin * z_vec
            ee_target = tip_target - R_fixed.dot(r_tip)
            poses.append(pose_from(ee_target, q_fixed))

        return poses, z_vec, q_fixed

    def retract_from_surface(self, current_q, current_tip, lift_dist=None, steps=12):
        """
        Retract the probe tip away from the surface along the current tool Z axis.
        """
        if lift_dist is None:
            lift_dist = self.approach_dist + self.retreat_dist

        q = np.asarray(current_q, float)
        R = tft.quaternion_matrix(q)[:3, :3]
        z = R[:, 2]
        r_tip = self.tip_vec_ee if self.tip_vec_ee is not None else np.array([0.0, 0.0, float(self.tip_to_contact)], float)

        tip_up = current_tip - lift_dist * z
        ee_up = tip_up - R.dot(r_tip)

        ee_now_pose = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([ee_now_pose.position.x, ee_now_pose.position.y, ee_now_pose.position.z], float)

        poses = [pose_from((1.0 - s) * p_now + s * ee_up, q)
                 for s in np.linspace(0.0, 1.0, max(2, steps))]
        return poses

    # ---------------- Pipeline ----------------
    def touch_and_sweep_fixedZ(self):
        """
        Orchestrate touch + raster sweep + retract.
        """
        P0, n0 = self.pick_P0_ahead_of_tip()
        if P0 is None:
            rospy.logerr("Unable to select P0 from the cloud.")
            return

        # --- Touch phase: approach to P0 with fixed-Z orientation ---
        cur = self.group.get_current_pose(self.ee_link).pose
        p_now = np.array([cur.position.x, cur.position.y, cur.position.z], float)
        q_now = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        poses_touch, _ = self.make_waypoints_fixedZ(p_now, q_now, P0)
        joints_seq, solved, _ = self.ik_for_all(poses_touch, self.ref_frame)
        if joints_seq is None:
            rospy.logerr("Insufficient IK coverage for the touch phase.")
            return

        rospy.loginfo("Executing touch with %d/%d waypoints.", solved, len(poses_touch))
        if not self.exec_joint_traj(joints_seq, self.step_time):
            rospy.logerr("Touch execution failed.")
            return

        # Orientation after touch is used as reference for minimal yaw changes
        cur = self.group.get_current_pose(self.ee_link).pose
        q_touch = np.array([cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w], float)

        if not self.raster_enable:
            rospy.logerr("raster_enable:=false (nothing to do).")
            return

        # --- Plan surface track (serpentine) ---
        if self.raster_style == "cross":
            track_pts = self.plan_serpentine_cross(P0, n0)
            style_str = "CROSS"
        else:
            track_pts = self.plan_serpentine_parallel(P0, n0)
            style_str = "PARALLEL"

        if track_pts is None:
            rospy.logerr("Raster planning failed (style=%s).", self.raster_style)
            return

        rospy.loginfo("Raster style=%s | pts=%d | sweep_length=%.3f | raster_width=%.3f | line_spacing=%.3f | samples/line=%d",
                      style_str, len(track_pts), float(self.sweep_length),
                      float(self.raster_width), float(self.raster_line_spacing),
                      int(self.samples_per_line))

        # --- Convert to poses and execute sweep ---
        poses_sweep, _, _ = self.poses_from_surface_track_fixedZ(q_touch, track_pts)
        joints_seq2, solved2, _ = self.ik_for_all(poses_sweep, self.ref_frame)
        if joints_seq2 is None:
            rospy.logerr("Insufficient IK coverage for sweep (solved %d/%d).", solved2, len(poses_sweep))
            return

        rospy.loginfo("Executing sweep with %d/%d waypoints.", solved2, len(poses_sweep))
        if not self.exec_joint_traj(joints_seq2, self.sweep_step_time):
            rospy.logerr("Sweep execution failed.")
            return

        # --- Retract from the surface ---
        last_pose = self.group.get_current_pose(self.ee_link).pose
        q_last = np.array([last_pose.orientation.x, last_pose.orientation.y,
                           last_pose.orientation.z, last_pose.orientation.w], float)
        R_last = tft.quaternion_matrix(q_last)[:3, :3]
        z_last = R_last[:, 2]
        tip_last = track_pts[-1] - self.sweep_contact_margin * z_last

        poses_up = self.retract_from_surface(
            q_last, tip_last,
            lift_dist=self.retreat_dist + self.approach_dist,
            steps=14
        )
        joints_seq3, solved3, _ = self.ik_for_all(poses_up, self.ref_frame)
        if joints_seq3 is not None:
            rospy.loginfo("Executing retract with %d/%d waypoints.", solved3, len(poses_up))
            self.exec_joint_traj(joints_seq3, self.step_time)
        else:
            rospy.logwarn("Retract not plannable, skipping.")

        # --- Return to pre-approach configuration ---
        self.step_to_joints(self.pre_joints, self.fallback_steps_pre, "pre_approach")
        rospy.loginfo("Sequence completed.")


if __name__ == "__main__":
    # Run node and always cleanly shutdown MoveIt bindings
    try:
        PreToPoseAndTouchFixedZ()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
