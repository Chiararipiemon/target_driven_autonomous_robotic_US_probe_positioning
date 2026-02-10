#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
us_pose_planner_confidence.py

Local pose planner driven by an Ultrasound confidence volume (US-only).

This node proposes a probe pose close to an apex point while maximizing visibility according to a
confidence volume stored in a .mha file. The output is a PoseStamped representing the best pose
in the same frame of the input point cloud.

What you must change to test this pipeline with your own setup:
1) Input topics:
   - ~cloud_topic must publish sensor_msgs/PointCloud2 for the skin surface
   - ~target_topic must publish geometry_msgs/PointStamped for the target
   - ~apex_pose_topic must publish geometry_msgs/PoseStamped for the apex pose
     The apex orientation x-axis is used as a yaw hint.
2) Confidence volume:
   - ~conf_mha_path must point to your confidence volume file (.mha)
   - ~conf_volume_frame must match how the volume is expressed ("imfusion" or "world")
   - ~T_imfusion_from_world_mm must match your calibration if using "imfusion"
3) Output topics:
   - ~best_pose_topic publishes the chosen PoseStamped
   - ~candidates_markers_topic publishes MarkerArray for RViz visualization
4) Runtime dependencies:
   - SciPy is required for cKDTree
   - SimpleITK is required to load the .mha volume (optional at import time, required at runtime if use_confidence is true)
5) Interactive mode:
   - ~interactive_mode can be "off" or "console"
     If you need console input, run under a terminal (for example: roslaunch with xterm prefix).

Inputs:
  /skin_cloud      (PointCloud2)   skin point cloud in world
  /us_target_viz   (PointStamped)  target in world
  /us_apex_pose    (PoseStamped)   apex pose in world (provides s0, z_base, x_hint)

Outputs:
  /us_best_pose         (PoseStamped)   chosen best pose
  /us_pose_candidates   (MarkerArray)   candidate pose visualization

Confidence volume is assumed to be in ImFusion physical coordinates (mm) by default.
Provide: ~conf_volume_frame:=imfusion and ~T_imfusion_from_world_mm accordingly.
"""

import rospy
import numpy as np
from scipy.spatial import cKDTree

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped
import sensor_msgs.point_cloud2 as pc2

import tf.transformations as tft
from visualization_msgs.msg import Marker, MarkerArray

import ast
import threading

try:
    import SimpleITK as sitk
except Exception:
    sitk = None


def _norm(v, eps=1e-12):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def _angle_deg(a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    a = a / na
    b = b / nb
    c = np.clip(np.dot(a, b), -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def _parse_param_matrix(val, fallback_4x4=None):
    """
    Accepts list-of-lists or a string like '[[...],[...],...]'.
    Returns np.array(float) with shape (4,4).
    """
    if fallback_4x4 is None:
        fallback_4x4 = np.eye(4).tolist()

    if val is None:
        return np.array(fallback_4x4, dtype=float)

    if isinstance(val, (list, tuple)):
        return np.array(val, dtype=float)

    if isinstance(val, str):
        try:
            parsed = ast.literal_eval(val)
            return np.array(parsed, dtype=float)
        except Exception:
            return np.array(fallback_4x4, dtype=float)

    return np.array(fallback_4x4, dtype=float)


def _parse_list(val, fallback=None):
    if fallback is None:
        fallback = []
    if val is None:
        return list(fallback)
    if isinstance(val, (list, tuple)):
        return list(val)
    if isinstance(val, str):
        try:
            parsed = ast.literal_eval(val)
            if isinstance(parsed, (list, tuple)):
                return list(parsed)
        except Exception:
            pass
    return list(fallback)


def _as_bool(v, default=False):
    if isinstance(v, bool):
        return v
    if v is None:
        return default
    return str(v).strip().lower() in ["1", "true", "yes", "y", "on"]


class ConfidenceVolume:
    """
    SimpleITK MHA reader + query in physical coordinates (mm).

    Query expects p_mm_xyz as (x,y,z) in millimeters.
    The internal array indexing is [z, y, x].
    """

    def __init__(self, mha_path, query_mode="trilinear", outside_value=0.0, auto_normalize=True):
        if sitk is None:
            raise RuntimeError("SimpleITK not available. Install with: python3 -m pip install --user SimpleITK")

        self.path = str(mha_path)
        self.query_mode = str(query_mode).lower()
        self.outside_value = float(outside_value)

        img = sitk.ReadImage(self.path)
        arr = sitk.GetArrayFromImage(img).astype(np.float32)

        self.vol_zyx = arr
        self.shape_zyx = arr.shape

        self.origin_mm = np.array(list(img.GetOrigin()), dtype=np.float64)
        self.spacing_mm = np.array(list(img.GetSpacing()), dtype=np.float64)
        self.direction = np.array(list(img.GetDirection()), dtype=np.float64).reshape(3, 3)
        self.inv_direction = np.linalg.inv(self.direction)

        if auto_normalize:
            vmax = float(np.nanmax(self.vol_zyx))
            if vmax > 1.01:
                if vmax <= 255.0 + 1e-3:
                    self.vol_zyx = self.vol_zyx / 255.0
                else:
                    self.vol_zyx = self.vol_zyx / (vmax + 1e-12)

        self.vmin = float(np.nanmin(self.vol_zyx))
        self.vmax = float(np.nanmax(self.vol_zyx))

    def _mm_to_continuous_index_xyz(self, p_mm_xyz):
        p = np.asarray(p_mm_xyz, dtype=np.float64)
        u = self.inv_direction.dot(p - self.origin_mm)
        idx = u / (self.spacing_mm + 1e-12)
        return idx

    def query_nn(self, p_mm_xyz):
        x, y, z = self._mm_to_continuous_index_xyz(p_mm_xyz)
        ix, iy, iz = np.round([x, y, z]).astype(int)
        Z, Y, X = self.shape_zyx
        if ix < 0 or iy < 0 or iz < 0 or ix >= X or iy >= Y or iz >= Z:
            return self.outside_value
        return float(self.vol_zyx[iz, iy, ix])

    def query_trilinear(self, p_mm_xyz):
        x, y, z = self._mm_to_continuous_index_xyz(p_mm_xyz)
        Z, Y, X = self.shape_zyx

        if (x < 0.0 or y < 0.0 or z < 0.0 or x > (X - 1) or y > (Y - 1) or z > (Z - 1)):
            return self.outside_value

        x0 = int(np.floor(x))
        x1 = min(x0 + 1, X - 1)
        y0 = int(np.floor(y))
        y1 = min(y0 + 1, Y - 1)
        z0 = int(np.floor(z))
        z1 = min(z0 + 1, Z - 1)

        xd = float(x - x0)
        yd = float(y - y0)
        zd = float(z - z0)

        c000 = self.vol_zyx[z0, y0, x0]
        c100 = self.vol_zyx[z0, y0, x1]
        c010 = self.vol_zyx[z0, y1, x0]
        c110 = self.vol_zyx[z0, y1, x1]
        c001 = self.vol_zyx[z1, y0, x0]
        c101 = self.vol_zyx[z1, y0, x1]
        c011 = self.vol_zyx[z1, y1, x0]
        c111 = self.vol_zyx[z1, y1, x1]

        c00 = c000 * (1 - xd) + c100 * xd
        c10 = c010 * (1 - xd) + c110 * xd
        c01 = c001 * (1 - xd) + c101 * xd
        c11 = c011 * (1 - xd) + c111 * xd

        c0 = c00 * (1 - yd) + c10 * yd
        c1 = c01 * (1 - yd) + c11 * yd

        c = c0 * (1 - zd) + c1 * zd
        return float(c)

    def query(self, p_mm_xyz):
        if self.query_mode in ["nn", "nearest"]:
            return self.query_nn(p_mm_xyz)
        return self.query_trilinear(p_mm_xyz)

    def stats(self):
        return {
            "shape_zyx": self.shape_zyx,
            "origin_mm": self.origin_mm.tolist(),
            "spacing_mm": self.spacing_mm.tolist(),
            "vmin": self.vmin,
            "vmax": self.vmax,
            "query_mode": self.query_mode,
            "path": self.path,
        }


class USPosePlannerConfidence(object):
    def __init__(self):
        self.target_topic = rospy.get_param("~target_topic", "/us_target_viz")
        self.apex_pose_topic = rospy.get_param("~apex_pose_topic", "/us_apex_pose")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/skin_cloud")
        self.best_pose_topic = rospy.get_param("~best_pose_topic", "/us_best_pose")

        self.interactive_mode = str(rospy.get_param("~interactive_mode", "off")).lower()
        self.max_user_options = int(rospy.get_param("~max_user_options", 4))

        self.patch_radius_m = float(rospy.get_param("~patch_radius_m", 0.008))
        self.n_apex_samples = int(rospy.get_param("~n_apex_samples", 12))

        self.shift_max_mm = float(rospy.get_param("~shift_max_mm", 35.0))
        self.delta_theta_max_deg = float(rospy.get_param("~delta_theta_max_deg", 18.0))
        self.alpha_max_deg = float(rospy.get_param("~alpha_max_deg", 20.0))

        self.theta_align_max_deg = float(rospy.get_param("~theta_align_max_deg", 15.0))
        self.theta_align_deg = float(rospy.get_param("~theta_align_deg", 12.0))
        self.theta_fov_half_deg = float(rospy.get_param("~theta_fov_half_deg", 15.0))

        self.beta_to_target = float(rospy.get_param("~beta_to_target", 0.5))
        self.tilt_jitter_deg = float(rospy.get_param("~tilt_jitter_deg", 10.0))
        self.n_dir_samples = int(rospy.get_param("~n_dir_samples", 15))

        self.n_yaw_samples = int(rospy.get_param("~n_yaw_samples", 2))
        self.yaw_deg_list = _parse_list(rospy.get_param("~yaw_deg_list", [0.0, 90.0]), fallback=[0.0, 90.0])
        self.use_yaw_list = _as_bool(rospy.get_param("~use_yaw_list", True), True)

        self.w_ray = float(rospy.get_param("~w_ray", 1.0))
        self.w_tgt = float(rospy.get_param("~w_tgt", 0.5))
        self.w_align = float(rospy.get_param("~w_align", 1.2))
        self.w_fov = float(rospy.get_param("~w_fov", 0.2))
        self.w_tilt = float(rospy.get_param("~w_tilt", 0.15))
        self.w_move = float(rospy.get_param("~w_move", 0.05))
        self.w_coll = float(rospy.get_param("~w_coll", 0.0))

        self.conf_mha_path = rospy.get_param("~conf_mha_path", "/tmp/volume_confidence.mha")
        self.conf_query_mode = rospy.get_param("~conf_query_mode", "trilinear")
        self.conf_outside_value = float(rospy.get_param("~conf_outside_value", 0.0))
        self.conf_volume_frame = str(rospy.get_param("~conf_volume_frame", "imfusion")).lower()
        self.use_confidence = _as_bool(rospy.get_param("~use_confidence", True), True)

        T_default = [
            [1.0, 0.0, 0.0, -642.0],
            [0.0, 0.0, 1.0, -364.0],
            [0.0, -1.0, 0.0, -200.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        self.T_imfusion_from_world_mm = _parse_param_matrix(
            rospy.get_param("~T_imfusion_from_world_mm", T_default),
            fallback_4x4=T_default,
        )

        self.ray_step_mm = float(rospy.get_param("~ray_step_mm", 2.0))
        self.conf_agg = str(rospy.get_param("~conf_agg", "mean")).lower()
        self.conf_p_quantile = float(rospy.get_param("~conf_p_quantile", 0.98))

        self.conf_target_radius_mm = float(rospy.get_param("~conf_target_radius_mm", 8.0))
        self.conf_target_samples = int(rospy.get_param("~conf_target_samples", 20))

        self.y_plane_max_mm = float(rospy.get_param("~y_plane_max_mm", 6.0))
        self.w_outplane = float(rospy.get_param("~w_outplane", 0.6))
        self.outplane_hard = _as_bool(rospy.get_param("~outplane_hard", False), False)

        self.publish_candidates_markers = _as_bool(rospy.get_param("~publish_candidates_markers", True), True)
        self.candidates_markers_topic = rospy.get_param("~candidates_markers_topic", "/us_pose_candidates")
        self.candidates_pub = rospy.Publisher(self.candidates_markers_topic, MarkerArray, queue_size=1, latch=True)

        self.cloud_frame_id = None
        self.cloud_points = None
        self.kdtree = None

        self.last_target = None
        self.last_apex_pos = None
        self.last_apex_z = None
        self.last_apex_x = None

        self.plan_done_for_current_target = False
        self.plan_lock = threading.Lock()
        self.planning_in_progress = False

        self.best_pose_pub = rospy.Publisher(self.best_pose_topic, PoseStamped, queue_size=1, latch=True)

        self.conf_vol = None
        self._init_confidence()

        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb)
        rospy.Subscriber(self.target_topic, PointStamped, self.target_cb)
        rospy.Subscriber(self.apex_pose_topic, PoseStamped, self.apex_pose_cb)

        rospy.loginfo("USPosePlannerConfidence ready. interactive_mode=%s", self.interactive_mode)

    def _init_confidence(self):
        if not self.use_confidence:
            rospy.logwarn("Confidence disabled by parameter.")
            return
        try:
            self.conf_vol = ConfidenceVolume(
                self.conf_mha_path,
                query_mode=self.conf_query_mode,
                outside_value=self.conf_outside_value,
                auto_normalize=True,
            )
            st = self.conf_vol.stats()
            rospy.loginfo(
                "Confidence volume loaded. shape(z,y,x)=%s v=[%.3f, %.3f] mode=%s",
                str(st["shape_zyx"]),
                st["vmin"],
                st["vmax"],
                st["query_mode"],
            )
        except Exception as e:
            rospy.logwarn("Cannot load confidence volume (%s): %s", self.conf_mha_path, str(e))
            self.conf_vol = None

    def pointcloud2_to_xyz_array(self, cloud_msg):
        pts = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([p[0], p[1], p[2]])
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    def cloud_cb(self, msg):
        self.cloud_frame_id = msg.header.frame_id
        self.cloud_points = self.pointcloud2_to_xyz_array(msg)
        if self.cloud_points.shape[0] == 0:
            rospy.logwarn("Skin cloud is empty, KD-tree not built.")
            self.kdtree = None
            return
        self.kdtree = cKDTree(self.cloud_points)
        rospy.loginfo_once("KD-tree built with %d points (frame: %s)", self.cloud_points.shape[0], self.cloud_frame_id)
        self.try_plan()

    def target_cb(self, msg):
        new_target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

        if self.last_target is None:
            self.last_target = new_target
        else:
            if np.linalg.norm(new_target - self.last_target) > 1e-4:
                self.plan_done_for_current_target = False
                self.last_target = new_target
                self.last_apex_pos = None
                self.last_apex_z = None
                self.last_apex_x = None

        self.try_plan()

    def apex_pose_cb(self, msg):
        self.last_apex_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float32)
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        R = tft.quaternion_matrix(q)[0:3, 0:3]
        self.last_apex_x = R[:, 0].astype(np.float32)
        self.last_apex_z = R[:, 2].astype(np.float32)
        self.try_plan()

    def sample_on_surface_patch(self, center, radius):
        if self.kdtree is None:
            return center.copy()
        idxs = self.kdtree.query_ball_point(center, r=radius)
        if not idxs:
            _, idx = self.kdtree.query(center)
            return self.cloud_points[idx].copy()
        k = np.random.choice(idxs)
        return self.cloud_points[k].copy()

    def estimate_normal_at(self, point):
        """PCA on neighbors in world. Returns a unit normal (sign ambiguous)."""
        if self.kdtree is None or self.cloud_points is None:
            return None
        k = min(30, self.cloud_points.shape[0])
        _, idxs = self.kdtree.query(point, k=k)
        neighbors = self.cloud_points[idxs, :] if not np.isscalar(idxs) else self.cloud_points[idxs:idxs + 1, :]
        c = neighbors.mean(axis=0)
        X = neighbors - c
        try:
            _, _, Vt = np.linalg.svd(X, full_matrices=False)
        except np.linalg.LinAlgError:
            return None
        n = Vt[-1, :]
        nrm = np.linalg.norm(n)
        return (n / nrm) if nrm > 1e-9 else None

    def world_to_imfusion_mm(self, p_world_m):
        p_world_m = np.asarray(p_world_m, dtype=float)
        p_world_mm = 1000.0 * p_world_m
        p_h = np.array([p_world_mm[0], p_world_mm[1], p_world_mm[2], 1.0], dtype=float)
        p_if = self.T_imfusion_from_world_mm @ p_h
        return p_if[:3]

    def query_confidence_world(self, p_world_m):
        if self.conf_vol is None:
            return None

        if self.conf_volume_frame == "imfusion":
            p_if_mm = self.world_to_imfusion_mm(p_world_m)
            return float(self.conf_vol.query(p_if_mm))

        if self.conf_volume_frame == "world":
            p_mm = 1000.0 * np.asarray(p_world_m, dtype=float)
            return float(self.conf_vol.query(p_mm))

        return None

    def ray_badness_from_confidence(self, s_world, T_world):
        """
        Returns a ray badness value in approximately [0,1]:
        higher means worse visibility (low confidence along the ray).
        Computed on bad = (1 - conf(p)).
        """
        if self.conf_vol is None:
            return None

        s = np.asarray(s_world, float)
        T = np.asarray(T_world, float)
        v = T - s
        L = np.linalg.norm(v)
        if L < 1e-9:
            return 0.0

        u = v / L
        step_m = self.ray_step_mm / 1000.0
        n = int(np.ceil(L / step_m))

        bad = []
        for i in range(n + 1):
            p = s + u * (i * step_m)
            c = self.query_confidence_world(p)
            if c is None:
                continue
            c = float(np.clip(c, 0.0, 1.0))
            bad.append(1.0 - c)

        if not bad:
            return None

        bad = np.asarray(bad, float)

        if self.conf_agg == "max":
            return float(np.max(bad))
        if self.conf_agg in ["p95", "quantile"]:
            return float(np.quantile(bad, self.conf_p_quantile))
        return float(np.mean(bad))

    def target_badness_from_confidence(self, T_world):
        """Returns g_tgt = 1 - mean(conf) in a small sphere around the target."""
        if self.conf_vol is None:
            return None

        T = np.asarray(T_world, float)
        r_mm = float(self.conf_target_radius_mm)

        if r_mm <= 1e-6 or self.conf_target_samples <= 0:
            c = self.query_confidence_world(T)
            if c is None:
                return None
            return float(1.0 - np.clip(c, 0.0, 1.0))

        acc = 0.0
        cnt = 0
        for _ in range(int(self.conf_target_samples)):
            d = np.random.normal(size=3)
            dn = np.linalg.norm(d)
            if dn < 1e-9:
                continue
            d /= dn
            rho = (np.random.rand() ** (1.0 / 3.0)) * r_mm
            offset_m = (rho / 1000.0) * d
            p = T + offset_m
            c = self.query_confidence_world(p)
            if c is None:
                continue
            acc += float(np.clip(c, 0.0, 1.0))
            cnt += 1

        if cnt == 0:
            return None

        mean_c = acc / cnt
        return float(1.0 - mean_c)

    def ray_badness_from_confidence_oriented(self, s_world, z_dir, yaw_deg, x_hint, T_world):
        """
        Oriented ray badness that makes yaw and tilt relevant.

        The target is projected into the probe scan plane (x-z plane of the candidate orientation).
        The ray is evaluated from the surface point to the projected target point.
        Returns (g_ray, y_off_mm) where y_off_mm is the absolute out-of-plane distance.
        """
        if self.conf_vol is None:
            return None, 0.0

        s = np.asarray(s_world, float)
        T = np.asarray(T_world, float)

        q = self.quat_from_z_and_xhint(z_dir, x_hint, yaw_deg=yaw_deg)
        R = tft.quaternion_matrix(q)[0:3, 0:3]
        x_axis = R[:, 0]
        y_axis = R[:, 1]
        z_axis = R[:, 2]

        v = T - s
        z_t = float(np.dot(v, z_axis))
        x_t = float(np.dot(v, x_axis))
        y_t = float(np.dot(v, y_axis))

        if z_t <= 1e-6:
            return 1.0, 1000.0 * abs(y_t)

        T_plane = s + x_t * x_axis + z_t * z_axis
        g_ray = self.ray_badness_from_confidence(s, T_plane)
        return g_ray, 1000.0 * abs(y_t)

    def quat_from_z_and_xhint(self, z_dir, x_hint, yaw_deg=0.0):
        """
        Builds a right-handed orientation frame using:
          z = z_dir (unit)
          x_base = projection of x_hint onto plane orthogonal to z
          y_base = z x x_base

        Then rotates x/y around z by yaw_deg.
        Returns quaternion (x, y, z, w).
        """
        z = _norm(z_dir)
        xh = _norm(x_hint)

        x_base = xh - np.dot(xh, z) * z
        if np.linalg.norm(x_base) < 1e-6:
            up = np.array([0.0, 0.0, 1.0], float)
            if abs(np.dot(up, z)) > 0.9:
                up = np.array([1.0, 0.0, 0.0], float)
            x_base = up - np.dot(up, z) * z
        x_base = _norm(x_base)
        y_base = _norm(np.cross(z, x_base))

        psi = np.deg2rad(float(yaw_deg))
        c = np.cos(psi)
        s = np.sin(psi)
        x = _norm(c * x_base + s * y_base)
        y = _norm(-s * x_base + c * y_base)

        Rm = np.eye(4)
        Rm[0:3, 0] = x
        Rm[0:3, 1] = y
        Rm[0:3, 2] = z
        return tft.quaternion_from_matrix(Rm)

    def generate_z_candidates(self, s, T, x_hint, z_base):
        """
        Generates candidate z directions.
        A base direction is computed by blending the local normal and the target direction:
          z0 = normalize((1-beta)*n_local + beta*target_dir)
        Then small random micro-tilts are applied around z0.
        """
        n_local = self.estimate_normal_at(s)
        if n_local is None:
            n_local = _norm(z_base)

        target_dir = _norm(T - s)

        beta = np.clip(self.beta_to_target, 0.0, 1.0)
        z0 = _norm((1.0 - beta) * _norm(n_local) + beta * target_dir)
        if np.linalg.norm(z0) < 1e-9:
            z0 = target_dir

        zs = []
        for _ in range(max(1, int(self.n_dir_samples))):
            alpha = np.random.uniform(0.0, max(0.0, self.tilt_jitter_deg))
            zs.append(self._tilt_random(z0, alpha))
        return zs

    def _tilt_random(self, n_base, alpha_deg):
        """Tilts n_base by alpha_deg around a random tangent axis."""
        n = _norm(n_base)
        alpha = np.deg2rad(alpha_deg)

        up = np.array([0.0, 0.0, 1.0], float)
        if abs(np.dot(up, n)) > 0.9:
            up = np.array([1.0, 0.0, 0.0], float)

        t1 = np.cross(up, n)
        t1 = t1 / (np.linalg.norm(t1) + 1e-12)
        t2 = np.cross(n, t1)

        phi = np.random.uniform(0.0, 2.0 * np.pi)
        q = (np.cos(alpha) * n + np.sin(alpha) * (np.cos(phi) * t1 + np.sin(phi) * t2))
        return _norm(q)

    def yaw_samples(self):
        """
        Returns yaw degrees list.
        If use_yaw_list is true, yaw_deg_list is used (and padded if needed).
        Otherwise, evenly spaced yaws in [0, 180) are generated.
        """
        if self.use_yaw_list and len(self.yaw_deg_list) >= 1:
            ys = [float(v) for v in self.yaw_deg_list]
            if len(ys) >= self.n_yaw_samples:
                return ys[:self.n_yaw_samples]
            while len(ys) < self.n_yaw_samples:
                ys.append(float(ys[-1]))
            return ys

        K = max(1, int(self.n_yaw_samples))
        return [float(k * (180.0 / K)) for k in range(K)]

    def evaluate_candidate(self, s, z_dir, yaw_deg):
        """Returns (J, terms) or (None, terms) if discarded by hard constraints."""
        T = self.last_target
        if T is None:
            return None, {}

        s0 = self.last_apex_pos
        z_base = self.last_apex_z
        x_hint = self.last_apex_x

        shift_m = float(np.linalg.norm(s - s0))
        shift_mm = 1000.0 * shift_m
        if shift_mm > self.shift_max_mm:
            return None, {"discard": "shift", "shift_mm": shift_mm}

        n_local = self.estimate_normal_at(s)
        if n_local is None:
            n_local = _norm(z_base)

        tilt_vs_normal_deg = _angle_deg(z_dir, n_local)
        tilt_vs_normal_deg = min(tilt_vs_normal_deg, 180.0 - tilt_vs_normal_deg)
        if tilt_vs_normal_deg > self.alpha_max_deg:
            return None, {"discard": "tilt", "tilt_vs_normal_deg": tilt_vs_normal_deg}

        delta_theta_deg = _angle_deg(z_dir, z_base)
        if delta_theta_deg > self.delta_theta_max_deg:
            return None, {"discard": "delta_theta", "delta_theta_deg": delta_theta_deg}

        target_dir = _norm(T - s)
        angle_fov = _angle_deg(z_dir, target_dir)
        if angle_fov > self.theta_align_max_deg:
            return None, {"discard": "align", "angle_fov": angle_fov}

        g_ray, y_off_mm = self.ray_badness_from_confidence_oriented(
            s_world=s,
            z_dir=z_dir,
            yaw_deg=yaw_deg,
            x_hint=x_hint,
            T_world=T,
        )
        if g_ray is None:
            g_ray = float(np.linalg.norm(T - s))

        if self.outplane_hard and (y_off_mm > self.y_plane_max_mm):
            return None, {"discard": "outplane", "y_off_mm": y_off_mm}

        g_outplane = (y_off_mm / max(1e-6, self.y_plane_max_mm)) ** 2

        g_tgt = self.target_badness_from_confidence(T)
        if g_tgt is None:
            g_tgt = 0.0

        g_align = (angle_fov / max(1e-6, self.theta_align_deg)) ** 2

        if angle_fov <= self.theta_fov_half_deg:
            g_fov = 0.0
        else:
            dtheta = angle_fov - self.theta_fov_half_deg
            g_fov = (dtheta / self.theta_fov_half_deg) ** 2

        alpha_soft = 0.7 * self.alpha_max_deg
        g_tilt = max(0.0, tilt_vs_normal_deg - alpha_soft) ** 2

        g_move = shift_m + np.deg2rad(delta_theta_deg)

        J = (
            self.w_ray * float(g_ray)
            + self.w_tgt * float(g_tgt)
            + self.w_align * float(g_align)
            + self.w_fov * float(g_fov)
            + self.w_tilt * float(g_tilt)
            + self.w_outplane * float(g_outplane)
            + self.w_move * float(g_move)
            + self.w_coll * 0.0
        )

        terms = dict(
            J=float(J),
            g_ray=float(g_ray),
            g_tgt=float(g_tgt),
            g_align=float(g_align),
            g_fov=float(g_fov),
            g_tilt=float(g_tilt),
            g_move=float(g_move),
            angle_fov_deg=float(angle_fov),
            shift_mm=float(shift_mm),
            delta_theta_deg=float(delta_theta_deg),
            tilt_vs_normal_deg=float(tilt_vs_normal_deg),
            yaw_deg=float(yaw_deg),
            y_off_mm=float(y_off_mm),
            g_outplane=float(g_outplane),
        )
        return float(J), terms

    def build_pose(self, s, z_dir, yaw_deg):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.cloud_frame_id

        pose.pose.position.x = float(s[0])
        pose.pose.position.y = float(s[1])
        pose.pose.position.z = float(s[2])

        qx, qy, qz, qw = self.quat_from_z_and_xhint(z_dir, self.last_apex_x, yaw_deg=yaw_deg)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def publish_candidates(self, candidates, best_index=0):
        """Publishes RViz markers for candidates: list of (J, s, z_dir, yaw_deg, terms)."""
        if not self.publish_candidates_markers:
            return
        if self.cloud_frame_id is None:
            return

        ma = MarkerArray()
        Js = [c[0] for c in candidates]
        Jmin = min(Js) if Js else 0.0
        Jmax = max(Js) if Js else 1.0
        denom = (Jmax - Jmin) if (Jmax - Jmin) > 1e-9 else 1.0

        now = rospy.Time.now()

        for i, (J, s, z_dir, yaw_deg, terms) in enumerate(candidates):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.cloud_frame_id
            m.ns = "us_pose_candidates"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD

            m.pose.position.x = float(s[0])
            m.pose.position.y = float(s[1])
            m.pose.position.z = float(s[2])

            qx, qy, qz, qw = self.quat_from_z_and_xhint(z_dir, self.last_apex_x, yaw_deg=yaw_deg)
            m.pose.orientation.x = float(qx)
            m.pose.orientation.y = float(qy)
            m.pose.orientation.z = float(qz)
            m.pose.orientation.w = float(qw)

            m.scale.x = 0.05
            m.scale.y = 0.004
            m.scale.z = 0.004

            if i == best_index:
                m.color.r = 0.1
                m.color.g = 0.9
                m.color.b = 0.1
                m.color.a = 1.0
            else:
                t = (J - Jmin) / denom
                m.color.r = float(0.9 * t + 0.1)
                m.color.g = float(0.9 * (1.0 - t) + 0.1)
                m.color.b = 0.1
                m.color.a = 0.6

            m.lifetime = rospy.Duration(0.0)
            ma.markers.append(m)

        self.candidates_pub.publish(ma)

    def console_select_candidate(self, candidates):
        max_k = min(self.max_user_options, len(candidates))
        if max_k <= 0:
            return None, None

        rospy.loginfo("Interactive console mode enabled (max options=%d).", max_k)

        for i in range(max_k):
            J, s, z_dir, yaw_deg, terms = candidates[i]

            self.publish_candidates(candidates, best_index=i)
            self.best_pose_pub.publish(self.build_pose(s, z_dir, yaw_deg))

            rospy.loginfo(
                "Option %d/%d | J=%.3f | g_ray=%.3f g_tgt=%.3f | angle=%.2f deg | yaw=%.1f deg | shift=%.1f mm | dtheta=%.1f deg | tilt=%.1f deg",
                i + 1,
                max_k,
                J,
                terms.get("g_ray", 0.0),
                terms.get("g_tgt", 0.0),
                terms.get("angle_fov_deg", 0.0),
                terms.get("yaw_deg", 0.0),
                terms.get("shift_mm", 0.0),
                terms.get("delta_theta_deg", 0.0),
                terms.get("tilt_vs_normal_deg", 0.0),
            )

            try:
                ans = input("Accept this pose? [y/n]: ").strip().lower()
            except EOFError:
                ans = "n"

            if ans in ["y", "yes", "s", "si"]:
                rospy.loginfo("Option %d accepted by user.", i + 1)
                return candidates[i], i

            rospy.loginfo("Option %d rejected.", i + 1)

        rospy.logwarn("No pose accepted. Please select a new target.")
        return None, None

    def try_plan(self):
        if self.kdtree is None or self.cloud_points is None:
            return
        if self.last_target is None:
            return
        if self.last_apex_pos is None or self.last_apex_z is None or self.last_apex_x is None:
            return

        with self.plan_lock:
            if self.plan_done_for_current_target or self.planning_in_progress:
                return
            self.planning_in_progress = True

        try:
            ok = self.plan_once()
        finally:
            with self.plan_lock:
                self.planning_in_progress = False
                self.plan_done_for_current_target = bool(ok)

    def plan_once(self):
        s0 = self.last_apex_pos
        T = self.last_target

        cA = self.query_confidence_world(s0)
        cT = self.query_confidence_world(T)
        rospy.loginfo(
            "CONF sanity: C(apex)=%.3f C(target)=%.3f frame=%s",
            -1.0 if cA is None else cA,
            -1.0 if cT is None else cT,
            self.conf_volume_frame,
        )

        p_if_apex = self.world_to_imfusion_mm(s0)
        p_if_tgt = self.world_to_imfusion_mm(T)
        rospy.loginfo(
            "IMF coords: apex_if_mm=(%.1f %.1f %.1f) target_if_mm=(%.1f %.1f %.1f)",
            p_if_apex[0], p_if_apex[1], p_if_apex[2],
            p_if_tgt[0], p_if_tgt[1], p_if_tgt[2],
        )

        yaws = self.yaw_samples()

        candidates = []
        discards = {"shift": 0, "tilt": 0, "delta_theta": 0, "align": 0, "outplane": 0}

        for _ in range(self.n_apex_samples):
            s = self.sample_on_surface_patch(s0, self.patch_radius_m)
            z_list = self.generate_z_candidates(s, T, self.last_apex_x, self.last_apex_z)

            for z_dir in z_list:
                for yaw_deg in yaws:
                    J, terms = self.evaluate_candidate(s, z_dir, yaw_deg)
                    if J is None:
                        reason = terms.get("discard", "other")
                        if reason in discards:
                            discards[reason] += 1
                        continue
                    candidates.append((J, s, z_dir, float(yaw_deg), terms))

        if not candidates:
            rospy.logwarn("No valid candidates. Discards=%s", str(discards))
            return False

        candidates.sort(key=lambda x: x[0])

        if self.interactive_mode == "console":
            selected, sel_idx = self.console_select_candidate(candidates)
            if selected is None:
                return False
            best = selected
            best_idx = sel_idx
        else:
            best = candidates[0]
            best_idx = 0
            self.publish_candidates(candidates, best_index=0)

        best_J, best_s, best_z, best_yaw, best_terms = best

        self.publish_candidates(candidates, best_index=best_idx)

        rospy.loginfo(
            "Best J=%.3f | g_ray=%.3f g_tgt=%.3f g_align=%.3f | angle=%.2f deg | yaw=%.1f deg | shift=%.1f mm | dtheta=%.1f deg | tilt=%.1f deg | discards=%s",
            best_J,
            best_terms.get("g_ray", 0.0),
            best_terms.get("g_tgt", 0.0),
            best_terms.get("g_align", 0.0),
            best_terms.get("angle_fov_deg", 0.0),
            best_terms.get("yaw_deg", 0.0),
            best_terms.get("shift_mm", 0.0),
            best_terms.get("delta_theta_deg", 0.0),
            best_terms.get("tilt_vs_normal_deg", 0.0),
            str(discards),
        )

        best_pose = self.build_pose(best_s, best_z, best_yaw)
        self.best_pose_pub.publish(best_pose)
        return True


def main():
    rospy.init_node("us_pose_planner_confidence")
    _ = USPosePlannerConfidence()
    rospy.spin()


if __name__ == "__main__":
    main()
