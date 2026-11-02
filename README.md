# Magnetron
The companion server to Mon-D, which handles the Autonomy and Navigation compute Mon-D can't.

# Mon-D mapping server

Server-side RTAB-Map launch and localization configuration, with the Mon-D model, camera mount, RoArm M3 visualization and joint-state helpers. Mon-D is the separate robot; its hardware code is already maintained in another [GitHub repository.](https://github.com/TogaTheGreat/Mon-D)

The server launcher consumes robot data and runs the external RTAB-Map backend. See [mapping/README.md](mapping/README.md) for the input topics, TF ownership, dependencies and historical helper scripts.

## Build and display

The workstation has ROS 2 Jazzy installed. Install colcon and rosdep for your ROS environment first, then from this repository:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch mondy_description display.launch.py
```

`depthai_descriptions` supplies the OAK-D-PRO mesh; install it in your ROS environment if rosdep cannot resolve it. The launch opens RViz and starts the joint-state defaults publisher. Its GUI is disabled by default; enable with `joint_state_gui:=true` for model inspection. Review joint defaults and encoder units before operating hardware.

## Mapping and localization

`mapping/RTAB_map_start.launch.py` is the rover RTAB-Map launch copied from `~/server`. It requires the RTAB-Map ROS packages, camera topics and rover TF/odometry. Inspect its arguments with:

```bash
ros2 launch ./mapping/RTAB_map_start.launch.py --show-args
```

The default map database is `~/.ros/mondy_map.db`; existing maps are excluded. The launch defaults to `base_footprint`, while the description uses `base_link`; select `base_frame:=base_link` if your live TF does not supply `base_footprint`. The populated EKF parameter file in `mapping/config` requires `robot_localization` and matching live sensor topics. It is not started by the mapping launch. The empty global EKF placeholder is omitted.

`config/cyclonedds_mondy.example.xml` preserves the local DDS configuration. Adapt interface `eno1` to your host before setting `CYCLONEDDS_URI`.

`legacy/joint_state_mapper.py` preserves the standalone `~/rb.py` mapper for reference. It is superseded by `joint_state_defaults.py`; do not run both publishers together. The optional mesh editor needs `numpy` and `trimesh`.

## Scope and upload

See [THIRD_PARTY.md](THIRD_PARTY.md) for attribution and licensing notices. Preparation notes are local-only and excluded from Git.