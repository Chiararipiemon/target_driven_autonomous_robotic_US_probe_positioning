# Simulation set-up and requirements
This repo works with Ubuntu 20.04 and ROS Noetic. Most of the files are connected and based on ***https://github.com/IFL-CAMP/iiwa_stack***.
With these scripts you are able to generate trajectories and let the robot perform the scans on the skin cloudpoint of the patient.
In this work there are two planned trajectories:
1. Linear along the spine
2. Serpentine
### Load iiwa_stack
#### Clone this repository to your workspace:
```
mkdir iiwa_stack_ws && cd iiwa_stack_ws && mkdir src
catkin_init_workspace
git clone https://github.com/IFL-CAMP/iiwa_stack.git src/iiwa_stack
```
#### Download the dependences :
```
rosdep install --from-paths src --ignore-src -r -y
```
#### Build the workspace :
```
catkin build
```
#### Source the workspace :
```
source devel/setup.bash
```
#### Load the directory:
Load the directory ***iiwa_probe_utils***. The directory should be inside iiwa_stack/src. Load also the directory ***cloudpoint*** inside iiwa_probe_utils.

## Spawning of manipulator + bed + robot pedestal + attach the probe with probe holder
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils setup_environment.launch
```
-----------------------------------------------------------------------------------------------------
## Load and spawn skin cloudpoint with anatomy segmentations inside 
```
source ~/iiwa_stack_ws/devel/setup.bash
roslaunch iiwa_probe_utils skin_and_segmentations.launch
```
If the point cloud doesn’t appear, go to the MoveIt! GUI → Add → PointCloud2 and select the topic: /cloud_with_normals. Then File --> save config

<img width="1092" height="626" alt="immagine" src="https://github.com/user-attachments/assets/9f4f0aa9-c033-4ae3-9a86-ea2d8b910c47" />


-----------------------------------------------------------------------------------------------------
After running your node for path execution, if you want to record everything ad get the spline for running the hybrid simulation:
## Record csv file [x,y,z,qx,qy,qz,qw]
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

## Execute raster scan
### Only touch the p0 point
- robot che va da home > pre.approach > si posiziona normale al punto p0: il contatto avviene tra frame probe_tip e punto p0.Il frame probe_tip è circa coincidente con la punta finale del probe.
```
ROS_NAMESPACE=iiwa \
~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/4_nov/pre_to_pose_and_touch.py \
  _group_name:=manipulator _ee_link:=iiwa_link_ee _ref_frame:=world \
  _speed_scale:=0.2 \
  _pre_joints:="[-2.529, 0.271, -0.268, 1.141, 2.932, 1.581, 0.174]" \
  _target_joints:="[-0.176, 0.675, 0.008, -0.789, -0.004, 1.669, -0.169]" \
  _cloud_topic:=/cloud_with_normals \
  _tip_frame:=probe_tip \
  _contact_margin:=0.006 \
  _approach_dist:=0.05 _retreat_dist:=0.00 \
  _far_steps:=1 _pre_steps:=12 _approach_steps:=18 \
  _step_time:=0.25 _ik_timeout:=1.5 \
  _allow_partial:=true _min_partial_fraction:=0.2 _pos_tol_final:=0.02 \
  _ik_service:=/iiwa/compute_ik
```
### Execute normal based raster scan for each point of the cloudpoint --> linear sweep
- Codice che fa scorrere il probe su una linea retta (sweep lineare) P0→Pdes, Z_tool = −normale in ogni campione, roll bloccato sulla direzione della linea, lift e rientro in pre_approach

Approfondimenti nella cartella 4_nov
```
cd /home/chiararipiemo/iiwa_stack_ws
source devel/setup.bash
ROS_NAMESPACE=/iiwa \
python3 src/iiwa_probe_utils/scripts/4_nov/pre_to_pose_touch_and_sweep_new.py \
  _cloud_topic:=/cloud_with_normals \
  _sweep_length:=0.20 \
  _sweep_samples:=40 \
  _sweep_pref_dir:='[0,1,0]' \
  _approach_dist:=0.03 \
  _retreat_dist:=0.06 \
  _ik_timeout:=3.0 \
  _allow_partial:=true \
  _min_partial_fraction:=0.10
```
### Execute no-normal based raster scan for each point of the cloudpoint (zfixed)--> linear sweep
- visti i precedenti probemi riscontrati nel precedente aggiornamento, sto lavorando ad un codice che tenga sempre fisso l'orientamento del probe e non consideri più la nromale ad ogni punto
```
ROS_NAMESPACE=/iiwa \
python3 src/iiwa_probe_utils/scripts/4_nov/pre_to_pose_touch_and_sweep_fixedZ.py \
  _cloud_topic:=/cloud_with_normals \
  _sweep_length:=0.20 \
  _sweep_samples:=40 \
  _sweep_pref_dir:='[0,1,0]' \
  _use_fixed_z:=true \
  _fixed_z_dir:='[0,0,-1]' \
  _fixed_flip_to_face_target:=true \
  _approach_dist:=0.03 \
  _retreat_dist:=0.06 \
  _ik_timeout:=3.0 \
  _allow_partial:=true \
  _min_partial_fraction:=0.10
```
### Execute serpentine raster scan
In this repo the code is inside Hybrid_simulation/raster
```
source ~/iiwa_stack_ws/devel/setup.bash

ROS_NAMESPACE=iiwa \
python3 ~/iiwa_stack_ws/src/iiwa_probe_utils/scripts/18Jan/raster_serpentine_scan_MLS.py \
  _cloud_topic:=/skin_cloud \
  _raster_enable:=true \
  _raster_style:=cross \
  _sweep_length:=0.20 \
  _raster_width:=0.06 \
  _raster_line_spacing:=0.02 \
  _samples_per_line:=20 \
  _raster_bridge_samples:=6 \
  _sweep_pref_dir:='[0,1,0]' \
  _fixed_z_dir:='[0,0,-1]' \
  _fixed_flip_to_face_target:=true \
  _approach_dist:=0.03 \
  _retreat_dist:=0.06 \
  _ik_timeout:=3.0 \
  _allow_partial:=true \
  _min_partial_fraction:=0.10 \
  _mls_enable:=true \
  _mls_k:=35 \
  _mls_alpha:=0.25 \
  _mls_reproject:=true

```
What I obtain with this code is something like this:
<img width="658" height="489" alt="image" src="https://github.com/user-attachments/assets/2fe2eb5d-4fed-4f90-b036-4ec80b12f49e" />

Then, inside console python imfusion:

```
import runpy; runpy.run_path("/home/chiararipiemo/iiwa_stack_ws/src/iiwa_probe_utils/Hybrid_simulation/sweeps_for_hus.py", run_name="__main__")
```
