# Setup
Follow this workflow just the first time.
To test the pipeline with synth data you need to segment from a Ct scan the anatomies of interest. For example you may need to segment:
- Spine
- kindey
- fat
Just use 3DSclicer.
N.B. Please pay attention to select the exact label number Imfusion wants to get the Hybrid Simulation:
<img width="1529" height="968" alt="image" src="https://github.com/user-attachments/assets/b52016c8-d041-4b59-b110-5042adbcc239" />

You need also to exract the skin surface and convert it in cloudpoint (pay attention, Rviz! supports few points, around 8000 it's fine)

## Spawning of manipulator + bed + robot pedestal + attach the probe with probe holder
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils setup_environment.launch
```
## Load and spawn skin cloudpoint with anatomy segmentations inside 
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils skin_and_segmentations.launch
```
If the point cloud doesn’t appear, go to the MoveIt! GUI → Add → PointCloud2 and select the topic: /cloud_with_normals. Then File --> save config

<img width="1092" height="626" alt="immagine" src="https://github.com/user-attachments/assets/9f4f0aa9-c033-4ae3-9a86-ea2d8b910c47" />

After running your node for path execution, if you want to record everything ad get the spline for running the hybrid simulation:
## Record csv file [x,y,z,qx,qy,qz,qw]
You need to record the probe poses to obtain the trajectory and convert it into the inputs for the hybrid simulation. SO:
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils csv_logger.launch
```
### Start
```
rosservice call /csv_logger/start
```
### Stop 
```
rosservice call /csv_logger/stop
```
The exit file will be a .csv file inside /iiwa_csv in this case.

--------------------------------------------------------------------------------------------------------------------------------------------------------------
## Plan and execute scanning
IMPORTANT!For simualtion only: trajectory tracking in joint space via waypoint IK, with a fixed tool-orientation constraint (fixed-Z), where the Cartesian path is generated offline-ish from a point cloud (plus smoothing/MLS), and then executed open-loop as a time-parameterized joint trajectory.There is no continuous feedback controller correcting the motion online based on measured contact/force/surface error during simulation execution because everything is already "perfect".

### Execute no-normal based raster scan for each point of the cloudpoint (zfixed)--> linear sweep
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils touch_p0_and_sweep_fixed_z.launch
```
### Execute serpentine raster scan

```
roslaunch iiwa_probe_utils raster_serpentine_scan_mls.launch

```
# Hybrid simulation
The idea is to start from the tracking sequence extracted from the .csv file, improve the trajectory by smoothing it if it is too zig-zagged, and generate two splines: a transducer spline and a direction spline.
First convert everything from Moveit! frame to ImFusion frame:

```
cd /home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/hybrid_simulation
python3 convert_to_imfusion_frame.py
```
Inside console python ImFusion:

```
import runpy; runpy.run_path("/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/hybrid_simulation/sweeps_for_hus.py", run_name="__main__")
```
Now you have two PolySplines but you still need two **GlSpline**. Unfortunatly there is now way right to obtain them directly by code but you need to modify the xml ImFusion file. Inside the directory hybrid_simulation there is an example for this modification.

## Move manually the splines from Global annotations to segmentation label
<img width="412" height="173" alt="immagine" src="https://github.com/user-attachments/assets/43bd2234-96d1-44a4-bf88-6981582aa329" />

Then you will have something like this:
<img width="1845" height="859" alt="immagine" src="https://github.com/user-attachments/assets/995b54dd-f164-49a8-8aa4-6ade12dc2bfe" />
Now you can run **Hybrid Ultrasound Simulation** inside ImFusionSuite

<img width="1114" height="455" alt="hbridsimulation" src="https://github.com/user-attachments/assets/29801c97-3f5e-4f9d-a45b-f4671ddd1f0e" />
