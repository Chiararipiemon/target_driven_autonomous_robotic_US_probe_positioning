# Probe pose planner
## Start node apex_projector
This node listens to a skin surface point cloud on ~cloud_topic and builds a KD-tree from the first received cloud to enable fast nearest-neighbor queries. When a target point arrives on ~target_topic, the node finds the closest skin point and either returns that nearest neighbor directly or, if ~use_plane_fit is enabled, fits a local plane to the ~k_neighbors nearest points and projects the target onto that plane to obtain a smoother and more stable “apex” on the surface. It publishes the resulting apex point on ~apex_topic and also publishes a pose on /us_apex_pose where the position is the apex and the orientation is constructed so that the tool Z axis points approximately inward by aligning it with the negative of the estimated surface normal.
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
