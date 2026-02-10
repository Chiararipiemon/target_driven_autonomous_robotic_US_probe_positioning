#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Apex projector node.
# This ROS node projects a user-defined ultrasound target point onto the skin surface
# represented by a point cloud (e.g., /skin_cloud). The output is:
# - /us_apex (PointStamped): the projected point on the skin surface
# - /us_apex_pose (PoseStamped): the same point plus an orientation derived from the local surface normal
# - /us_target_viz (PointStamped): republishes the input target for RViz visualization

import rospy
import numpy as np
from scipy.spatial import cKDTree

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft


class ApexProjector(object):
    def __init__(self):
        # Topic providing the desired ultrasound target point (in the same frame as the cloud, ideally).
        self.target_topic = rospy.get_param("~target_topic", "/us_target")

        # Topic providing the skin surface point cloud (PointCloud2).
        self.cloud_topic = rospy.get_param("~cloud_topic", "/skin_cloud")

        # Output topic for the projected apex point on the skin.
        self.apex_topic = rospy.get_param("~apex_topic", "/us_apex")

        # Number of neighbors used to estimate a local tangent plane (PCA/SVD plane fit).
        self.k_neighbors = int(rospy.get_param("~k_neighbors", 30))

        # If True, use local plane fitting + projection; if False, use nearest-neighbor only.
        self.use_plane_fit = bool(rospy.get_param("~use_plane_fit", True))

        # Publisher for the projected apex point (latched so RViz can display it even if published once).
        self.apex_pub = rospy.Publisher(self.apex_topic, PointStamped, queue_size=1, latch=True)

        # Publisher for the apex pose (position + orientation).
        # The orientation is built so that the tool Z axis points inward (approximately along -normal).
        self.apex_pose_pub = rospy.Publisher("/us_apex_pose", PoseStamped, queue_size=1, latch=True)

        # Publisher used only for RViz visualization of the incoming target.
        self.target_viz_pub = rospy.Publisher("/us_target_viz", PointStamped, queue_size=1, latch=True)

        # Last target used for visualization and for duplicate-filtering.
        self.last_target_viz = None
        self.last_target = None

        # If the new target is closer than this threshold to the previous one, ignore it (avoid spam).
        self.target_eps = float(rospy.get_param("~target_eps", 1e-4))  # meters

        # Cloud storage.
        self.cloud_frame_id = None
        self.cloud_points = None  # Nx3 float array
        self.kdtree = None        # KD-tree built over cloud_points

        # Subscribers.
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb)
        rospy.Subscriber(self.target_topic, PointStamped, self.target_cb)

        # Optional: periodically republish the last target for RViz.
        # Useful if RViz subscribes later or if you want it refreshed at a fixed rate.
        # self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo(
            "ApexProjector initialized. Waiting for skin cloud on %s and targets on %s",
            self.cloud_topic, self.target_topic
        )

    def pointcloud2_to_xyz_array(self, cloud_msg):
        """
        Convert a ROS PointCloud2 message into an (N, 3) numpy array [x, y, z].
        NaNs are skipped.
        """
        points_list = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([p[0], p[1], p[2]])

        if len(points_list) == 0:
            return np.empty((0, 3), dtype=np.float32)

        return np.asarray(points_list, dtype=np.float32)

    def cloud_cb(self, msg):
        """
        Skin cloud callback.

        This node builds a KD-tree only once (on the first received cloud) to avoid
        re-building at every message and spamming logs.

        If you expect the cloud to change over time and you want updates, you can
        remove the early return and rebuild periodically or whenever needed.
        """
        # If already initialized, do not rebuild again.
        if self.kdtree is not None and self.cloud_points is not None:
            return

        rospy.loginfo_once("Received first skin cloud on %s", self.cloud_topic)

        self.cloud_frame_id = msg.header.frame_id
        self.cloud_points = self.pointcloud2_to_xyz_array(msg)

        if self.cloud_points.shape[0] == 0:
            rospy.logwarn("Skin cloud is empty, cannot build KD-tree.")
            self.kdtree = None
            self.cloud_points = None
            return

        # Build KD-tree once.
        self.kdtree = cKDTree(self.cloud_points)
        rospy.loginfo(
            "KD-tree built with %d points (frame: %s)",
            self.cloud_points.shape[0], self.cloud_frame_id
        )

    def project_on_local_plane(self, target, idx_nearest):
        """
        Project the target point onto a locally estimated plane around the nearest neighbor.

        Steps:
        1) Query k nearest neighbors around the target from the KD-tree.
        2) Fit a plane to neighbors via PCA (SVD). The last singular vector is the normal.
        3) Project the target onto the plane.
        4) Orient the normal so that the tool Z axis can be chosen consistently.

        Returns
        -------
        apex : np.ndarray (3,)
            The projected point on the local plane (proxy for skin surface).
        normal : np.ndarray (3,) or None
            Local surface normal. The code flips it so that -normal points approximately
            toward the target (i.e., into the patient) for consistent tool orientation.
        """
        k = min(self.k_neighbors, self.cloud_points.shape[0])

        if k <= 1:
            # Not enough points for a plane fit; fallback to nearest neighbor.
            return self.cloud_points[idx_nearest].copy(), None

        dists, idxs = self.kdtree.query(target, k=k)

        # Ensure we always have a (k,3) neighbor array.
        if np.isscalar(idxs):
            neighbors = self.cloud_points[idxs:idxs + 1, :]
        else:
            neighbors = self.cloud_points[idxs, :]

        # Plane centroid.
        c = neighbors.mean(axis=0)

        # PCA/SVD for normal estimation: normal is the smallest-variance direction.
        X = neighbors - c
        try:
            _, _, Vt = np.linalg.svd(X, full_matrices=False)
        except np.linalg.LinAlgError:
            rospy.logwarn("SVD failed, falling back to nearest neighbor apex.")
            return self.cloud_points[idx_nearest].copy(), None

        normal = Vt[-1, :]
        nrm = np.linalg.norm(normal)
        if nrm < 1e-9:
            rospy.logwarn("Degenerate normal, falling back to nearest neighbor apex.")
            return self.cloud_points[idx_nearest].copy(), None

        normal = normal / nrm

        # Project target onto the plane defined by (c, normal).
        v = target - c
        dist_to_plane = np.dot(v, normal)
        apex = target - dist_to_plane * normal

        # Normal orientation heuristic:
        # We want the tool Z axis to be aligned with -normal and to point roughly inward.
        # The vector (target - apex) points from the skin toward the original target.
        v_apex_target = target - apex
        if np.linalg.norm(v_apex_target) > 1e-9:
            u = v_apex_target / np.linalg.norm(v_apex_target)
            dot = np.dot(normal, u)

            # If normal points toward the target (dot > 0), then -normal points away from it.
            # Flip so that -normal points toward the target direction consistently.
            if dot > 0.0:
                normal = -normal

        return apex, normal

    def quat_from_normal(self, normal):
        """
        Create an orientation quaternion where the tool Z axis points along -normal.

        Convention used:
        - z_axis = -normal (points "inward")
        - choose an 'up' reference to define x_axis in a stable way
        - compute orthonormal basis (x, y, z) and convert to quaternion
        """
        n = np.asarray(normal, float)
        if np.linalg.norm(n) < 1e-9:
            return 0.0, 0.0, 0.0, 1.0

        # Tool Z axis points inward.
        z = -n / np.linalg.norm(n)

        # Choose an up vector that is not parallel to z.
        up = np.array([0.0, 0.0, 1.0], float)
        if abs(np.dot(z, up)) > 0.9:
            up = np.array([1.0, 0.0, 0.0], float)

        # Build orthonormal basis.
        x = np.cross(up, z)
        x = x / (np.linalg.norm(x) + 1e-12)
        y = np.cross(z, x)

        # Rotation matrix as columns (x, y, z).
        R = np.eye(4)
        R[:3, :3] = np.column_stack((x, y, z))
        qx, qy, qz, qw = tft.quaternion_from_matrix(R)
        return qx, qy, qz, qw

    def target_cb(self, msg):
        """
        Target callback.

        When a new target arrives:
        1) Publish /us_target_viz for RViz.
        2) Find nearest neighbor on the skin cloud via KD-tree.
        3) Optionally estimate a local plane and project the target onto it.
        4) Publish /us_apex as the projected point.
        5) Publish /us_apex_pose with an orientation derived from the surface normal.
        """
        if self.kdtree is None or self.cloud_points is None:
            rospy.logwarn_throttle(5.0, "KD-tree not ready yet, waiting for valid skin cloud.")
            return

        target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

        # Ignore very small target changes.
        if self.last_target is not None:
            if np.linalg.norm(target - self.last_target) < self.target_eps:
                return
        self.last_target = target.copy()

        # Publish target for visualization.
        target_viz = PointStamped()
        target_viz.header.stamp = rospy.Time.now()
        target_viz.header.frame_id = self.cloud_frame_id if self.cloud_frame_id else msg.header.frame_id
        target_viz.point.x = float(target[0])
        target_viz.point.y = float(target[1])
        target_viz.point.z = float(target[2])

        self.last_target_viz = target_viz
        self.target_viz_pub.publish(target_viz)

        # Nearest neighbor query.
        dist_nn, idx_nn = self.kdtree.query(target)

        # Compute apex and normal.
        normal = None
        if self.use_plane_fit:
            apex_xyz, normal = self.project_on_local_plane(target, idx_nn)
            dist = np.linalg.norm(apex_xyz - target)
        else:
            apex_xyz = self.cloud_points[idx_nn]
            dist = dist_nn

        # Publish apex point.
        apex_msg = PointStamped()
        apex_msg.header.stamp = rospy.Time.now()
        apex_msg.header.frame_id = self.cloud_frame_id
        apex_msg.point.x = float(apex_xyz[0])
        apex_msg.point.y = float(apex_xyz[1])
        apex_msg.point.z = float(apex_xyz[2])
        self.apex_pub.publish(apex_msg)

        # Publish apex pose (position + orientation).
        apex_pose = PoseStamped()
        apex_pose.header.stamp = rospy.Time.now()
        apex_pose.header.frame_id = self.cloud_frame_id
        apex_pose.pose.position.x = apex_msg.point.x
        apex_pose.pose.position.y = apex_msg.point.y
        apex_pose.pose.position.z = apex_msg.point.z

        # If we have a normal, build orientation from it.
        # Otherwise, fallback to using the direction (target - apex) as a pseudo-normal.
        if normal is not None:
            qx, qy, qz, qw = self.quat_from_normal(normal)
        else:
            dir_vec = target - apex_xyz
            if np.linalg.norm(dir_vec) < 1e-6:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx, qy, qz, qw = self.quat_from_normal(-dir_vec)

        apex_pose.pose.orientation.x = qx
        apex_pose.pose.orientation.y = qy
        apex_pose.pose.orientation.z = qz
        apex_pose.pose.orientation.w = qw
        self.apex_pose_pub.publish(apex_pose)

        rospy.loginfo(
            "Apex computed: dist=%.3f m | target=(%.3f, %.3f, %.3f) -> apex=(%.3f, %.3f, %.3f) [k=%d, plane_fit=%s]",
            dist,
            target[0], target[1], target[2],
            apex_xyz[0], apex_xyz[1], apex_xyz[2],
            self.k_neighbors, self.use_plane_fit
        )


def main():
    rospy.init_node("apex_projector")
    ApexProjector()
    rospy.spin()


if __name__ == "__main__":
    main()

