#!/usr/bin/env python3
"""
Publishes RViz arrow markers that visualize per-point normals from a PointCloud2.
It subscribes to a cloud containing x,y,z and normal_x,normal_y,normal_z fields, decimates the points,
normalizes the normals, and publishes a MarkerArray in the same frame as the input cloud.

What you may want to tweak
- ~topic_in / ~topic_out: input cloud topic and output marker topic
- ~scale: arrow length in meters
- ~step: publish 1 arrow every N points (basic decimation)
- ~max_markers: hard cap on the number of arrows (auto-increases the step if needed)
"""

import rospy, numpy as np, math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def make_arrow(p, n, frame_id, mid, scale=0.02, color=(0.1,0.8,1.0,0.9)):
    """
    Create a single ARROW marker from point p and unit normal n.
    The arrow starts at p and ends at p + n*scale, in the given frame_id.
    """
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "cloud_normals"
    m.id = mid
    m.type = Marker.ARROW
    m.action = Marker.ADD

    # Marker.ARROW uses scale as: x = shaft diameter, y = head diameter, z = head length
    # Here we keep a compact arrow; the actual arrow length is given by the two points.
    m.scale.x = scale * 0.6  # shaft diameter (not length)
    m.scale.y = scale * 0.2  # head diameter
    m.scale.z = scale * 0.2  # head length

    m.color.r, m.color.g, m.color.b, m.color.a = color

    s = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
    e = Point(x=float(p[0] + n[0]*scale), y=float(p[1] + n[1]*scale), z=float(p[2] + n[2]*scale))
    m.points = [s, e]
    return m

class NormalsMarkers:
    def __init__(self):
        # Topics and visualization settings
        self.topic_in    = rospy.get_param("~topic_in", "/cloud_with_normals")
        self.scale       = float(rospy.get_param("~scale", 0.02))      # arrow length [m]
        self.step        = int(rospy.get_param("~step", 50))           # 1 arrow every N points
        self.max_markers = int(rospy.get_param("~max_markers", 500))   # upper bound on arrows
        self.topic_out   = rospy.get_param("~topic_out", "/cloud_normals_markers")

        # latch=True so RViz gets the last published MarkerArray immediately when it connects
        self.pub = rospy.Publisher(self.topic_out, MarkerArray, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.topic_in, PointCloud2, self.cb, queue_size=1)
        rospy.loginfo("Waiting for cloud on %s ...", self.topic_in)

    def cb(self, msg: PointCloud2):
        # Use the cloud frame to avoid TF mismatches in RViz
        fid = msg.header.frame_id or "world"

        # Required fields in the PointCloud2 message (keep order fixed)
        fields = ['x','y','z','normal_x','normal_y','normal_z']
        has_needed = all(any(f.name == req for f in msg.fields) for req in fields)
        if not has_needed:
            rospy.logerr("Cloud does not have required fields: %s", fields)
            return

        # Read points and normals, skipping NaNs
        pts, nrms = [], []
        for rec in pc2.read_points(msg, field_names=fields, skip_nans=True):
            x, y, z, nx, ny, nz = rec
            pts.append([x, y, z])
            nrms.append([nx, ny, nz])

        if not pts:
            rospy.logwarn("Empty cloud received.")
            return

        P = np.asarray(pts, float)
        N = np.asarray(nrms, float)

        # Normalize normals to unit vectors (avoid division by zero)
        nn = np.linalg.norm(N, axis=1, keepdims=True)
        nn[nn == 0] = 1.0
        N = N / nn

        # Controlled decimation:
        # start from user step, then enforce max_markers by increasing step if needed
        step = max(1, self.step)
        if self.max_markers > 0:
            step = max(step, int(math.ceil(len(P) / float(self.max_markers))))

        arr = MarkerArray()
        mid = 0
        for i in range(0, len(P), step):
            arr.markers.append(make_arrow(P[i], N[i], fid, mid, scale=self.scale))
            mid += 1

        self.pub.publish(arr)
        rospy.loginfo(
            "Published %d normal arrows on %s (frame=%s, step=%d, scale=%.3f)",
            len(arr.markers), self.topic_out, fid, step, self.scale
        )

def main():
    rospy.init_node("normals_markers_from_cloud")
    NormalsMarkers()
    rospy.spin()

if __name__ == "__main__":
    main()
