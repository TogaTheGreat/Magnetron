# Server mapping

This repository runs on the server. Mon-D is the separate robot publishing camera data, wheel/EKF odometry and TF; its hardware code is already maintained in a different GitHub repository.

## Current files

- `RTAB_map_start.launch.py`: server RTAB-Map mapping/localization launch. Starts the SLAM node, with optional RGB-D odometry, visualization and topic diagnostics.
- `config/mondy_ekf.yaml`: populated wheel-odometry EKF parameters. The original `global_mondy_ekf.yaml` contained no configuration and is omitted.
- `legacy/rtab_test.sh`: older standalone raw-camera mapping diagnostic, retained from the server sources.
- `legacy/commands.txt`: historical two-machine bringup notes; paths and manual TF commands describe older arrangements.

## Inputs and ownership

The current launcher subscribes to `/rgbd_image/compressed` and `/odometry/filtered` (the latter can be changed with `odom_topic`). It does not start the camera, RGB-D sync/compression, robot drivers or an EKF node. Those inputs and their TF chain must already exist. `run_rgbd_odometry:=true` adds diagnostic visual odometry on `/rgbd/odom` with TF publishing disabled.

Defaults are `base_footprint`, `odom` and `map`. The robot description has `base_link`; use the actual live TF chain, or override `base_frame:=base_link` when appropriate. RTAB-Map publishes map TF when `publish_tf:=true`. Avoid competing TF publishers when using the historical commands or diagnostic script. `camera_frame`, `oak_root_frame` and `publish_camera_bridge_tf` are retained launch arguments, but the current launcher does not create a camera bridge TF from them.

The EKF file consumes `/wheel/odom`, publishes TF, and uses `base_footprint` as its base frame. Its IMU input is commented out. It is not loaded by the RTAB-Map launch; only run a server EKF if the robot is not already supplying that function. The copied comment about conditional EKF TF ownership was corrected to describe this behavior.

## Use

From the repository root, after sourcing ROS 2 and the built description workspace:

```bash
ros2 launch ./mapping/RTAB_map_start.launch.py --show-args
ros2 launch ./mapping/RTAB_map_start.launch.py database_path:="$HOME/.ros/mondy_map.db"
```

The default database path is `~/.ros/mondy_map.db`. The current launch preserves existing databases; `localization:=true` uses an existing map without adding nodes. Map databases are runtime data and excluded from Git.

For server visualization:

```bash
ros2 launch mondy_description display.launch.py
```

RTAB-Map and RViz depend on working ROS discovery between the server and Mon-D. `config/cyclonedds_mondy.example.xml` at the repository root preserves the workstation's interface-specific DDS example. The original workstation also sets `ROS_STATIC_PEERS=Mon-D.local`; adapt ROS discovery settings to the deployed RMW and network.

## Legacy diagnostic

`legacy/rtab_test.sh` is not the current server launcher. It starts its own static camera TF, RGB-D synchronization, visual odometry, SLAM and GUI. It includes `--delete_db_on_start`, which resets RTAB-Map's default database. Its `ros2 topic hz` checks are continuous and require interruption; it is preserved as historical source rather than a verified unattended bringup script. Do not run it alongside the current TF/SLAM pipeline.

No additional custom mapping backend was found in the workstation project sources: the main algorithm is supplied by the external RTAB-Map packages.
