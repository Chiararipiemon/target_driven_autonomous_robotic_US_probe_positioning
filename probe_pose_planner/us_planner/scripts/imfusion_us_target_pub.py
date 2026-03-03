#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped

# stessa matrice dell'export (mm)
T_IMFUSION_FROM_ROBOT_MM = np.array([
    [1.0, 0.0, 0.0, -642.0],
    [0.0, 0.0, 1.0, -364.0],
    [0.0, -1.0, 0.0, -200.0],
    [0.0, 0.0, 0.0,    1.0]
], dtype=float)

T_ROBOT_FROM_IMFUSION_MM = np.linalg.inv(T_IMFUSION_FROM_ROBOT_MM)

def publish_us_target_world_from_imfusion_mm(target_if_mm):
    # ImFusion(mm) -> world(mm)
    p_if = np.array([target_if_mm[0], target_if_mm[1], target_if_mm[2], 1.0], dtype=float)
    p_world_mm = T_ROBOT_FROM_IMFUSION_MM @ p_if
    p_world_m  = p_world_mm[:3] / 1000.0

    pub = rospy.Publisher("/us_target", PointStamped, queue_size=1)

    # Tempo per registrare il publisher al master
    rospy.sleep(0.2)

    msg = PointStamped()
    msg.header.frame_id = "world"
    msg.point.x = float(p_world_m[0])
    msg.point.y = float(p_world_m[1])
    msg.point.z = float(p_world_m[2])

    # Aspetta che apex_projector si connetta (handshake TCPROS)
    t0 = rospy.Time.now()
    timeout = rospy.Duration(1.0)
    r = rospy.Rate(100)
    while pub.get_num_connections() == 0 and (rospy.Time.now() - t0) < timeout and not rospy.is_shutdown():
        r.sleep()

    # Pubblica più volte per essere sicuri
    for _ in range(10):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.sleep(0.02)

    rospy.loginfo("Published /us_target world(m)=(%.6f, %.6f, %.6f) subs=%d",
                  msg.point.x, msg.point.y, msg.point.z, pub.get_num_connections())

def main():
    rospy.init_node("imfusion_us_target_publisher", anonymous=True)

    if len(sys.argv) == 4:
        x_mm = float(sys.argv[1])
        y_mm = float(sys.argv[2])
        z_mm = float(sys.argv[3])
        target_if_mm = np.array([x_mm, y_mm, z_mm], dtype=float)
    else:
        rospy.logwarn("No args given. Using fallback target in mm.")
        target_if_mm = np.array([21.0759, -76.0087, -31.7101], dtype=float)

    publish_us_target_world_from_imfusion_mm(target_if_mm)

    # Rimani vivo un attimo, così la connessione non muore “subito”
    rospy.sleep(0.5)

if __name__ == "__main__":
    main()
