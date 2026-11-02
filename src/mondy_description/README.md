# mondy_description

This is a patched RViz/real-robot version of the JPL OSR Xacro.

Changes from the uploaded `osr.urdf.xacro`:
- Removed the Gazebo include (`gazebo.urdf.xacro`). It is only for simulation/ros2_control and is not needed for RViz or real TF.
- Kept `insert_transmission.urdf.xacro` because the OSR Xacro calls transmission macros at the bottom.
- Replaced mesh paths with `package://mondy_description/meshes/osr/...`.
- Removed the `base_footprint -> base_link` fixed joint so EKF can own `odom -> base_link` without TF conflicts.

The OSR STL files are included in:
  mondy_description/meshes/osr/

Expected mesh names:
  arm.stl
  body.stl
  body_axis.stl
  box_link.stl
  rockerbogie_front_1.stl
  rockerbogie_middle_1.stl
  turn_L.stl
  turn_R.stl
  wheel.stl
  wheel_link_left_2.stl
  wheel_middle_left.stl

Build:
  cd ~/Mon-D-github
  colcon build --symlink-install --packages-select mondy_description
  source install/setup.bash

Run test:
  ros2 launch mondy_description display.launch.py joint_state_gui:=true

For real rover usage, run with `joint_state_gui:=false` and provide real /joint_states:
  ros2 launch mondy_description display.launch.py joint_state_gui:=false

See the repository README for dependencies, the default joint-state publisher, and hardware setup notes.
