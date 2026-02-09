#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Description
Attaches a mesh (tool) to a robot link as an AttachedCollisionObject via MoveIt PlanningSceneInterface.

Two modes are supported:
1) Manual pose: set x/y/z + roll/pitch/yaw (deg) in the parent link frame.
2) TF pose: align the mesh to a TF frame (pose_frame) expressed in link_name, then apply an
   optional offset (translation + RPY deg) in the pose_frame coordinates.

To prevent MoveIt from reporting START_STATE_IN_COLLISION when the attached mesh intersects
nearby links, you can provide a list of allowed touch links via:
  ~touch_links: ["iiwa_link_7", "iiwa_link_ee", ...]

Usage (example)
  ROS_NAMESPACE=iiwa rosrun iiwa_probe_utils attach_tool_mesh.py \
    _mesh_path:=/abs/path/to/tool.dae _link_name:=iiwa_link_ee _name:=probe_holder \
    _x:=0 _y:=0 _z:=-0.11 _roll_deg:=0 _pitch_deg:=0 _yaw_deg:=0 \
    _touch_links:="['iiwa_link_7','iiwa_link_ee']"

What to change for your own setup
- mesh_path: path to your CAD mesh (.dae/.stl)
- link_name: the robot link to attach the tool to
- name: object id in PlanningScene
- pose: either manual (x/y/z + rpy deg) or TF-based (pose_from_tf + pose_frame + offsets)
- scale: if your mesh units differ (keep 1.0 if already meters)
- touch_links: links allowed to collide with the attached tool without invalidating the start state
- moveit_ns: set if your MoveIt interfaces live under a specific namespace (e.g., "/iiwa")
"""

import os
import sys
import math
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    PlanningSceneInterface,
    RobotCommander,
    roscpp_initialize,
    roscpp_shutdown,
)


def quat_from_rpy(roll, pitch, yaw):
    # Quaternion from RPY (rad), returns (x,y,z,w)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def normalize_moveit_ns(ns: str) -> str:
    # Convert "iiwa" or "/iiwa/" -> "/iiwa"
    if ns is None:
        return ""
    ns = ns.strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    ns = ns.rstrip("/")
    return ns


def wait_for_state_update(scene, name, attached=True, timeout=5.0):
    # Poll PlanningSceneInterface until the object appears attached (or removed).
    start = rospy.get_time()
    while (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([name])
        is_attached = len(attached_objects.keys()) > 0
        if attached and is_attached:
            return True
        if (not attached) and (name not in scene.get_known_object_names()):
            return True
        rospy.sleep(0.1)
    return False


def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("attach_tool_mesh", anonymous=True)

    # --- params ---
    mesh_path = os.path.expanduser(rospy.get_param("~mesh_path", ""))
    if not mesh_path or not os.path.exists(mesh_path):
        rospy.logerr("Missing or invalid ~mesh_path: %s", mesh_path)
        sys.exit(1)

    tool_name = rospy.get_param("~name", "probe_holder")
    link_name = rospy.get_param("~link_name", "iiwa_link_ee")

    # Manual pose (default)
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 0.0))
    roll_deg = float(rospy.get_param("~roll_deg", 0.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg = float(rospy.get_param("~yaw_deg", 0.0))

    # TF-based pose
    pose_from_tf = bool(rospy.get_param("~pose_from_tf", False))
    pose_frame = rospy.get_param("~pose_frame", "")

    off_x = float(rospy.get_param("~offset_x", 0.0))
    off_y = float(rospy.get_param("~offset_y", 0.0))
    off_z = float(rospy.get_param("~offset_z", 0.0))
    off_roll_deg = float(rospy.get_param("~offset_roll_deg", 0.0))
    off_pitch_deg = float(rospy.get_param("~offset_pitch_deg", 0.0))
    off_yaw_deg = float(rospy.get_param("~offset_yaw_deg", 0.0))

    # Mesh scaling
    sx = float(rospy.get_param("~scale_x", 1.0))
    sy = float(rospy.get_param("~scale_y", 1.0))
    sz = float(rospy.get_param("~scale_z", 1.0))

    # Touch links for attached collision object
    touch_links = rospy.get_param("~touch_links", [])
    if not isinstance(touch_links, list):
        rospy.logwarn("~touch_links is not a list. Ignoring.")
        touch_links = []
    if len(touch_links) == 0:
        # Safe default: allow collision with the parent link.
        touch_links = [link_name]

    # MoveIt namespace (optional)
    moveit_ns_param = rospy.get_param("~moveit_ns", "")
    if moveit_ns_param:
        moveit_ns = normalize_moveit_ns(moveit_ns_param)
    else:
        # Use current node namespace, e.g., "/iiwa"
        moveit_ns = normalize_moveit_ns(rospy.get_namespace())

    # --- MoveIt interfaces ---
    scene = PlanningSceneInterface(ns=moveit_ns)
    _robot = RobotCommander(ns=moveit_ns)  # ensures robot_description is reachable
    rospy.sleep(1.0)

    # --- cleanup old objects ---
    try:
        scene.remove_attached_object(link_name, name=tool_name)
        scene.remove_world_object(name=tool_name)
    except Exception:
        pass
    wait_for_state_update(scene, tool_name, attached=False, timeout=2.0)

    # --- tool pose relative to link_name ---
    pose = PoseStamped()
    pose.header.frame_id = link_name
    pose.header.stamp = rospy.Time.now()

    if pose_from_tf and pose_frame:
        tf_buf = tf2_ros.Buffer(rospy.Duration(5.0))
        tf_lis = tf2_ros.TransformListener(tf_buf)

        try:
            # pose_frame expressed in link_name
            tfm = tf_buf.lookup_transform(link_name, pose_frame, rospy.Time(0), rospy.Duration(2.0))
        except Exception as e:
            rospy.logerr("TF lookup failed (%s -> %s): %s", link_name, pose_frame, str(e))
            sys.exit(2)

        tx, ty, tz = tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z
        qx, qy, qz, qw = (
            tfm.transform.rotation.x,
            tfm.transform.rotation.y,
            tfm.transform.rotation.z,
            tfm.transform.rotation.w,
        )

        # Apply translation offset in pose_frame coordinates
        R = tft.quaternion_matrix([qx, qy, qz, qw])[:3, :3]
        off = R.dot([off_x, off_y, off_z])

        pose.pose.position.x = tx + float(off[0])
        pose.pose.position.y = ty + float(off[1])
        pose.pose.position.z = tz + float(off[2])

        # Apply rotational offset
        r = math.radians(off_roll_deg)
        p = math.radians(off_pitch_deg)
        yv = math.radians(off_yaw_deg)
        q_off = quat_from_rpy(r, p, yv)
        q_tot = tft.quaternion_multiply([qx, qy, qz, qw], q_off)

        pose.pose.orientation.x = q_tot[0]
        pose.pose.orientation.y = q_tot[1]
        pose.pose.orientation.z = q_tot[2]
        pose.pose.orientation.w = q_tot[3]

        rospy.loginfo("Attaching '%s' to %s aligned to TF frame '%s' (with offsets).",
                      tool_name, link_name, pose_frame)
    else:
        # Manual pose in link_name frame
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        yv = math.radians(yaw_deg)
        qx, qy, qz, qw = quat_from_rpy(r, p, yv)

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        rospy.loginfo("Attaching '%s' to %s using manual pose.", tool_name, link_name)

    # --- attach mesh (with touch_links) ---
    rospy.loginfo("touch_links=%s | scale=(%.3f, %.3f, %.3f) | moveit_ns='%s'",
                  touch_links, sx, sy, sz, moveit_ns)

    scene.attach_mesh(
        link_name,
        tool_name,
        pose=pose,
        filename=mesh_path,
        size=(sx, sy, sz),
        touch_links=touch_links,
    )

    ok = wait_for_state_update(scene, tool_name, attached=True, timeout=5.0)
    if not ok:
        rospy.logwarn("Could not verify attachment in time. Check RViz / PlanningScene.")
    else:
        rospy.loginfo("Tool attached successfully: %s", tool_name)

    roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
