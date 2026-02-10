# Probe pose planner
## Start node apex_projector
This node listens to a skin surface point cloud on ~cloud_topic and builds a KD-tree from the first received cloud to enable fast nearest-neighbor queries. When a target point arrives on ~target_topic, the node finds the closest skin point and either returns that nearest neighbor directly or, if ~use_plane_fit is enabled, fits a local plane to the ~k_neighbors nearest points and projects the target onto that plane to obtain a smoother and more stable “apex” on the surface. It publishes the resulting apex point on ~apex_topic and also publishes a pose on /us_apex_pose where the position is the apex and the orientation is constructed so that the tool Z axis points approximately inward by aligning it with the negative of the estimated surface normal.
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch us_planner apex_projector.launch
```
## Select the target
### Open ImFusion and copy this inside the python console
Inside ImFusionSuite the user can select a target point by running inside the python console:

```
import imfusion, subprocess; code = """
am = imfusion.app.annotation_model
ann = am.create_annotation(imfusion.Annotation.AnnotationType.POINT)
ann.color = (1.0, 0.0, 0.0)

def on_done():
    if not ann.points:
        print('No points defined')
        return
    x_mm, y_mm, z_mm = ann.points[0]
    print('ImFusion clicked point (mm):', x_mm, y_mm, z_mm)
    cmd = [
        'bash', '-lc',
        'source /opt/ros/noetic/setup.bash && source ~/iiwa_stack_ws/devel/setup.bash && rosrun us_planner imfusion_us_target_pub.py {} {} {}'.format(x_mm, y_mm, z_mm)
    ]
    subprocess.Popen(cmd)
    print('Sent point to ROS via rosrun (bash -lc)')

ann.on_editing_finished(on_done)
ann.start_editing()
print('Now click one POINT in the view to define the US target.')
"""; exec(code)
```
## Go to /us_apex_pose
```
roslaunch us_planner go_to_us_apex.launch
```
<img width="947" height="458" alt="immagine" src="https://github.com/user-attachments/assets/38130032-97e4-4984-bb59-70d2c553b1df" />

### Record probe pose onto apex point 
```
roslaunch us_planner record_us_apex_pose_imfusion_csv.launch
```
The output is something like this:

x_mm,y_mm,z_mm,qx,qy,qz,qw

10.465045,-40.403463,-158.472498,0.479116757,0.437854871,-0.555232485,0.520045319

## Start planner
The USPosePlannerConfidence is a local pose planner that proposes a probe pose near the current apex point by optimizing visibility in an ultrasound confidence volume.

It samples candidate contact points on the skin around the apex, builds candidate probe orientations by blending the local skin normal with the direction to the target, and optionally samples multiple yaw angles to allow different view directions (e.g., sagittal vs transverse-like). Each candidate is filtered by “sonographer-safe” hard constraints (limited shift and limited angular change), then scored using confidence-based costs (low confidence along the beam path and around the target), plus soft penalties for excessive motion, misalignment, and out-of-plane target distance. The best-scoring candidate is published as a PoseStamped on /us_best_pose, and the node can optionally show alternatives and ask the user to accept/reject them in a console-driven mode
```
roslaunch us_planner us_pose_planner_confidence.launch
```
## Go to /us_best_pose
```
roslaunch us_planner go_to_us_best_pose.launch
```
## Record probe pose onto us_best_pose
```
roslaunch us_planner record_us_best_pose_imfusion_csv.launch
```
