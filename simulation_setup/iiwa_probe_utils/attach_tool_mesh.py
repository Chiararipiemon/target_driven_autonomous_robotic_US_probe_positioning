#!/usr/bin/env python3
import os, sys, math, rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_commander import (
    MoveGroupCommander, PlanningSceneInterface,
    RobotCommander, roscpp_initialize, roscpp_shutdown
)

def quat_from_rpy(roll, pitch, yaw):
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qx,qy,qz,qw)

def wait_for_state_update(scene, name, attached=True, timeout=5.0):
    start = rospy.get_time()
    while (rospy.get_time() - start) < timeout and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([name])
        is_attached = len(attached_objects.keys()) > 0
        if attached and is_attached: return True
        if not attached and name not in scene.get_known_object_names(): return True
        rospy.sleep(0.1)
    return False

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node("attach_tool_mesh", anonymous=True)

    mesh_path = os.path.expanduser(rospy.get_param("~mesh_path", ""))
    if not mesh_path or not os.path.exists(mesh_path):
        rospy.logerr("~mesh_path mancante o file inesistente: %s", mesh_path); sys.exit(1)

    tool_name  = rospy.get_param("~name", "probe_holder")
    link_name  = rospy.get_param("~link_name", "iiwa_link_ee")

    # Modalità 1: pose manuale (default, come prima)
    x = float(rospy.get_param("~x", 0.0))
    y = float(rospy.get_param("~y", 0.0))
    z = float(rospy.get_param("~z", 0.0))
    roll_deg  = float(rospy.get_param("~roll_deg", 0.0))
    pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
    yaw_deg   = float(rospy.get_param("~yaw_deg", 0.0))

    # Modalità 2: pose da TF (nuovo)
    pose_from_tf = bool(rospy.get_param("~pose_from_tf", False))
    pose_frame   = rospy.get_param("~pose_frame", "")         # es. "probe_tip"
    # Offset opzionali, espressi nel frame 'pose_frame'
    off_x = float(rospy.get_param("~offset_x", 0.0))
    off_y = float(rospy.get_param("~offset_y", 0.0))
    off_z = float(rospy.get_param("~offset_z", 0.0))
    off_roll_deg  = float(rospy.get_param("~offset_roll_deg", 0.0))
    off_pitch_deg = float(rospy.get_param("~offset_pitch_deg", 0.0))
    off_yaw_deg   = float(rospy.get_param("~offset_yaw_deg", 0.0))

    sx = float(rospy.get_param("~scale_x", 1.0))
    sy = float(rospy.get_param("~scale_y", 1.0))
    sz = float(rospy.get_param("~scale_z", 1.0))

    ns = rospy.get_namespace().rstrip('/') or "iiwa"
    scene = PlanningSceneInterface(ns="/"+ns)
    robot = RobotCommander(ns="/"+ns)
    group = MoveGroupCommander("manipulator", ns="/"+ns)
    rospy.sleep(1.0)

    # pulizia oggetti precedenti
    try:
        scene.remove_attached_object(link_name, name=tool_name)
        scene.remove_world_object(name=tool_name)
    except Exception:
        pass
    wait_for_state_update(scene, tool_name, attached=False, timeout=2.0)

    # Pose del tool relativa al link_name
    pose = PoseStamped()
    pose.header.frame_id = link_name
    pose.header.stamp = rospy.Time.now()

    if pose_from_tf and pose_frame:
        # Allinea il mesh al frame 'pose_frame' (espresso in link_name)
        tf_buf = tf2_ros.Buffer(rospy.Duration(5.0))
        tf_lis = tf2_ros.TransformListener(tf_buf)
        try:
            tfm = tf_buf.lookup_transform(link_name, pose_frame, rospy.Time(0), rospy.Duration(2.0))
        except Exception as e:
            rospy.logerr("lookup_transform(%s->%s) fallita: %s", link_name, pose_frame, str(e))
            sys.exit(2)

        # traslazione base
        tx, ty, tz = tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z
        # rotazione base
        qx, qy, qz, qw = (tfm.transform.rotation.x, tfm.transform.rotation.y,
                          tfm.transform.rotation.z, tfm.transform.rotation.w)

        # applica offset (pos in pose_frame, quindi ruota in link_name)
        R = tft.quaternion_matrix([qx,qy,qz,qw])[:3,:3]
        off = R.dot([off_x, off_y, off_z])
        pose.pose.position.x = tx + off[0]
        pose.pose.position.y = ty + off[1]
        pose.pose.position.z = tz + off[2]

        # applica offset rotazionale
        r = math.radians(off_roll_deg)
        p = math.radians(off_pitch_deg)
        yv = math.radians(off_yaw_deg)
        q_off = quat_from_rpy(r, p, yv)
        q_tot = tft.quaternion_multiply([qx,qy,qz,qw], q_off)
        pose.pose.orientation.x, pose.pose.orientation.y = q_tot[0], q_tot[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q_tot[2], q_tot[3]

        rospy.loginfo("Attach mesh '%s' a %s allineato a frame '%s' (con offset).", tool_name, link_name, pose_frame)
    else:
        # comportamento originale: usa x,y,z e rpy
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        yv = math.radians(yaw_deg)
        qx,qy,qz,qw = quat_from_rpy(r, p, yv)

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        rospy.loginfo("Attach mesh '%s' a %s con pose manuale.", tool_name, link_name)

    scene.attach_mesh(link_name, tool_name, pose, mesh_path, size=(sx,sy,sz))
    ok = wait_for_state_update(scene, tool_name, attached=True, timeout=5.0)
    if not ok:
        rospy.logwarn("Non ho potuto verificare l'attacco. Controlla in RViz.")
    rospy.loginfo("Done.")
    roscpp_shutdown()

if __name__ == "__main__":
    main()

