A key requirement of the simulation environment was to reproduce the behavior of an ultrasound machine, displaying the corresponding US image for each probe pose as the robot scans the surface. To achieve this, a software bridge between RViz and ImFusion was developed, enabling the probe pose in the MoveIt simulation to drive the real-time visualization of ultrasound images within the ImFusion interface using the \textit{Hybrid Ultrasound Simulation}. Due to some software limitations, the computed slice is always linear. Future implementation will include the use of the SDK for computing the slices also with convex geometry.

1. open ImFusionSuite
2. open python console
3. run
```
exec(open("YOOUR_PATH/iiwa_probe_utils/hybrd_simulation_single_slice/get_sweep_from_csv_quaternion.py", encoding="utf-8").read()); main()
```
4. run the hybrid simulation

<img width="572" height="750" alt="hybridflow" src="https://github.com/user-attachments/assets/9249d2be-c410-4f11-a98c-0e01bfef85a2" />
