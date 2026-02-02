# Target-Driven Autonomous Robotic US Probe Positioning for Anatomies Occluded by Acoustic Shadows
The goal of this work is to develop a robotic ultrasound application that autonomously optimizes the pose of an ultrasound probe to improve the visibility of an operator-selected anatomical target
that is partially or fully hidden by acoustic shadowing.
The key idea is to combine robotized data acquisition and a planning strategy, so that the robot can safely approach the skin with a probe pose orientation that maximizes the expected target visibility.

## Sistem overview
<img width="813" height="770" alt="sysoverview" src="https://github.com/user-attachments/assets/90a410f8-c326-4c85-bce5-4a5e2e73f70f" />

Figure summarizes the proposed two-stage workflow of the robotic ultrasound system. In Phase 1, the setup is first calibrated using the external RGB-D camera and fiducial markers. From the acquired depth data, the patient skin surface and its normal direction are estimated, then the robot plans and executes one or more preliminary scans to collect ultrasound data. These scans are processed to generate a confidence map and a partial 3D volume reconstruction, which the operator inspects to annotate and select the target point.
In Phase 2, the selected target is projected onto the estimated skin surface to define an apex point and initialize the optimization-based planner. The planner computes the best probe pose, which is then converted into collision-aware robot
motions for safe execution under ROS Noetic coordination. During contact and scanning, compliant behavior is ensured via a PI admittance/impedance control strategy, while a dedicated Z-safety policy limits motion along the surface normal
to avoid excessive penetration. If the resulting pose quality is not satisfactory, the procedure iterates until a suitable pose is found.

## Repo overview
Inside **/simulation_setup** based on https://github.com/IFL-CAMP/iiwa_stack there are all the scripts and the README.md to run the simulation: all the inputs you need and all the outputs you should obtain by following the intructions.
Inside **/segmentation** you find all the custom scripts and priors to run the network for US automatic spine segmentation based on https://github.com/luohwu/UltraBones100k
Inside **/confidence_map** you find all the custom scripts for running the confidence map computation based on https://github.com/Project-MONAI/MONAI
Inside **/probe_pose_planner** there are all the custom scripts for running the probe US pose optimization problem 
Inside **/validation there** there is a README.md where you can find all the instructions to run and execute the entire pipeline and test everything
