#!/usr/bin/env python3
"""
This node attaches a tool mesh (STL/DAE/etc.) to a robot link in the MoveIt planning scene.

Why this exists
You often want the planner to "know" about the real end-effector geometry (probe holder, gripper, sensor body)
so collisions and reachability are computed with the correct shape. This script attaches a mesh as an *attached
collision object* to a given link (by default the iiwa end-effector link).

How the pose is defined
You have two ways to place the mesh relative to the target link:

1) Manual pose (default)
   You set x,y,z and roll/pitch/yaw (degrees) as ROS parameters. Pose is expressed in link_name.

2) Pose from TF (optional)
   If pose_from_tf is true, the script looks up a TF transform from link_name to pose_frame (e.g. "probe_tip"),
   aligns the mesh to that frame, and optionally applies an extra offset (translation and rotation) expressed in
   pose_frame coordinates.

Parameters you will likely edit
- ~mesh_path: absolute path to the mesh file (required)
- ~link_name: the link to attach the mesh to (default "iiwa_link_ee")
- ~name: collision object name in MoveIt (default "probe_holder")
- Manual mode: ~x ~y ~z ~roll_deg ~pitch_deg ~yaw_deg
- TF mode: ~pose_from_tf ~pose_frame plus optional offsets:
  ~offset_x ~offset_y ~offset_z ~offset_roll_deg ~offset_pitch_deg ~offset_yaw_deg
- Scaling: ~scale_x ~scale_y ~scale_z (useful if your mesh units are not meters)

Notes
- This uses MoveIt’s PlanningSceneInterface under the namespace inferred from rospy.get_namespace().
  If you launch it under /iiwa, it will talk to MoveIt services/topics under /iiwa.
- If an object with the same name already exists, it is removed first to avoid duplicates.
"""

import os, sys, math, rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    MoveGroupCommander, PlanningSceneInterface,
    RobotCommander, roscpp_initialize, roscpp_shutdown
)

def quat_from_rpy(roll, pitch, yaw):
    """
    Build a quaternion (x,y,z,w) from roll/pitch/yaw in radians.

    MoveIt and geometry_msgs use quaternions, while it’s convenient to tune tool orientation in RPY.
    This is a simple explicit conversion without relying on external helpers.
    """
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx,qy,qz,qw)

def wait_for_state_update(scene, name, attached=True, timeout=5.0):
    """
    Wait until the planning scene reflects the requested state for the object `name`.

    This helps with the fact that PlanningSceneInterface updates are asynchronous:
    you can call attach/remove and RViz/MoveIt might see it a moment later.
    """
    start = rospy.get_time()
    while (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([name])
        is_attached = len(attached_objects.keys()) > 0
        if attached and is_attached:
            return True
        if not attached and name not in scene.get_known_object_names():
            return True
        rospy.sleep(0.1)
    return False

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("attach_tool_mesh", anonymous=True)

    # Mesh path is mandatory: fail early if missing or invalid.
    mesh_path = os.path.expanduser(rospy.get_param("~mesh_path", ""))
    if not mesh_path or not os.path.exists(mesh_path):
        rospy.logerr("~mesh_path missing or file does not exist: %s", mesh_path)
        sys.exit(1)

    # MoveIt object name and robot link to attach to.
    tool_name  = rospy.get_param("~name", "probe_holder")
    link_name  = rospy.get_param("~link_name", "iiwa_link_ee")

    # Mode 1: manual pose (default)
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 0.0))
    roll_deg  = float(rospy.get_param("~roll_deg", 0.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg   = float(rospy.get_param("~yaw_deg", 0.0))

    # Mode 2: pose from TF (optional)
    pose_from_tf = bool(rospy.get_param("~pose_from_tf", False))
    pose_frame   = rospy.get_param("~pose_frame", "")  # for example: "probe_tip"

    # Optional offsets expressed in pose_frame coordinates.
    # Translation is rotated into link_name when applied.
    off_x = float(rospy.get_param("~offset_x", 0.0))
    off_y = float(rospy.get_param("~offset_y", 0.0))
    off_z = float(rospy.get_param("~offset_z", 0.0))
    off_roll_deg  = float(rospy.get_param("~offset_roll_deg", 0.0))
    off_pitch_deg = float(rospy.get_param("~offset_pitch_deg", 0.0))
    off_yaw_deg   = float(rospy.get_param("~offset_yaw_deg", 0.0))

    # Mesh scaling. Keep at 1.0 if your mesh is already in meters.
    sx = float(rospy.get_param("~scale_x", 1.0))
    sy = float(rospy.get_param("~scale_y", 1.0))
    sz = float(rospy.get_param("~scale_z", 1.0))

    # Infer MoveIt namespace from the ROS namespace the node is launched in.
    # Example: if launched under /iiwa, ns becomes "iiwa" and MoveIt is expected under /iiwa/*
    ns = rospy.get_namespace().rstrip('/') or "iiwa"
    scene = PlanningSceneInterface(ns="/"+ns)
    robot = RobotCommander(ns="/"+ns)
    group = MoveGroupCommander("manipulator", ns="/"+ns)
    rospy.sleep(1.0)

    # Cleanup: remove any previous object with the same name, both attached and in the world.
    # This avoids stale collision objects when relaunching.
    try:
        scene.remove_attached_object(link_name, name=tool_name)
        scene.remove_world_object(name=tool_name)
    except Exception:
        pass
    wait_for_state_update(scene, tool_name, attached=False, timeout=2.0)

    # Pose of the mesh relative to link_name.
    pose = PoseStamped()
    pose.header.frame_id = link_name
    pose.header.stamp = rospy.Time.now()

    if pose_from_tf and pose_frame:
        # TF mode: align the mesh to pose_frame, expressed in link_name.
        # Typical use: pose_frame is a tip frame you already publish (e.g. probe_tip).
        tf_buf = tf2_ros.Buffer(rospy.Duration(5.0))
        tf_lis = tf2_ros.TransformListener(tf_buf)
        try:
            tfm = tf_buf.lookup_transform(link_name, pose_frame, rospy.Time(0), rospy.Duration(2.0))
        except Exception as e:
            rospy.logerr("lookup_transform(%s -> %s) failed: %s", link_name, pose_frame, str(e))
            sys.exit(2)

        # Base translation from TF
        tx, ty, tz = tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z

        # Base rotation from TF (quaternion)
        qx, qy, qz, qw = (tfm.transform.rotation.x, tfm.transform.rotation.y,
                          tfm.transform.rotation.z, tfm.transform.rotation.w)

        # Apply translational offset expressed in pose_frame:
        # rotate offset by the TF rotation so it becomes a vector in link_name, then add to base translation.
        R = tft.quaternion_matrix([qx, qy, qz, qw])[:3, :3]
        off = R.dot([off_x, off_y, off_z])
        pose.pose.position.x = tx + off[0]
        pose.pose.position.y = ty + off[1]
        pose.pose.position.z = tz + off[2]

        # Apply rotational offset (RPY in degrees) on top of the TF orientation.
        r = math.radians(off_roll_deg)
        p = math.radians(off_pitch_deg)
        yv = math.radians(off_yaw_deg)
        q_off = quat_from_rpy(r, p, yv)

        # Total orientation: base * offset
        q_tot = tft.quaternion_multiply([qx, qy, qz, qw], q_off)
        pose.pose.orientation.x, pose.pose.orientation.y = q_tot[0], q_tot[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q_tot[2], q_tot[3]

        rospy.loginfo("Attaching mesh '%s' to %s aligned to TF frame '%s' (with offsets).",
                      tool_name, link_name, pose_frame)

    else:
        # Manual mode: use x,y,z and roll/pitch/yaw provided by parameters, expressed in link_name.
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

        rospy.loginfo("Attaching mesh '%s' to %s using manual pose parameters.",
                      tool_name, link_name)

    # Attach the mesh to the robot link as an attached collision object.
    # After this, MoveIt will consider the tool geometry during planning/collision checking.
    scene.attach_mesh(link_name, tool_name, pose, mesh_path, size=(sx, sy, sz))

    ok = wait_for_state_update(scene, tool_name, attached=True, timeout=5.0)
    if not ok:
        rospy.logwarn("Could not verify attachment. Check in RViz / Planning Scene.")
    rospy.loginfo("Done.")
    roscpp_shutdown()

if __name__ == "__main__":
    main()
