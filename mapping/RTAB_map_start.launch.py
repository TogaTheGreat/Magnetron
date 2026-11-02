#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---- Launch args ----
    base_frame = LaunchConfiguration("base_frame")
    oak_root_frame = LaunchConfiguration("oak_root_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    publish_camera_bridge_tf = LaunchConfiguration("publish_camera_bridge_tf")

    wait_for_transform = LaunchConfiguration("wait_for_transform")
    publish_tf = LaunchConfiguration("publish_tf")
    odom_topic = LaunchConfiguration("odom_topic")

    database_path = LaunchConfiguration("database_path")
    localization = LaunchConfiguration("localization")
    use_odom_guess = LaunchConfiguration("use_odom_guess")
    run_rgbd_odometry = LaunchConfiguration("run_rgbd_odometry")
    odom_image_decimation = LaunchConfiguration("odom_image_decimation")
    backend_detection_rate = LaunchConfiguration("backend_detection_rate")
    use_viz = LaunchConfiguration("use_viz")
    debug_checks = LaunchConfiguration("debug_checks")
    odom_reset_countdown = LaunchConfiguration("odom_reset_countdown")
    grid_max_ground_height = LaunchConfiguration("grid_max_ground_height")
    grid_max_ground_angle = LaunchConfiguration("grid_max_ground_angle")
    grid_flat_obstacles_detected = LaunchConfiguration("grid_flat_obstacles_detected")
    grid_max_obstacle_height = LaunchConfiguration("grid_max_obstacle_height")
    grid_prob_miss = LaunchConfiguration("grid_prob_miss")

    args = [

        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("oak_root_frame", default_value="oak"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),

        DeclareLaunchArgument("wait_for_transform", default_value="0.2"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),

        # Keep the old database intact; start the new odometry session separately.
        DeclareLaunchArgument(
            "database_path", default_value="~/.ros/mondy_map.db"),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Use the existing database for localization only. The default "
                "continues mapping while also allowing relocalization."
            ),
        ),
        DeclareLaunchArgument(
            "use_odom_guess",
            default_value="false",
            choices=["true", "false"],
            description="Seed RGB-D odometry from the wheel/EKF odom transform.",
        ),
        DeclareLaunchArgument(
            "run_rgbd_odometry",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Optional diagnostic visual odometry only. It never owns base TF "
                "and is not the navigation odometry source."
            ),
        ),
        DeclareLaunchArgument(
            "odom_image_decimation",
            default_value="1",
            description=(
                "Downsample RGB-D images by this integer factor before visual "
                "registration. This reduces local CPU, not network bandwidth."
            ),
        ),
        DeclareLaunchArgument(
            "backend_detection_rate",
            default_value="1.0",
            description="RTAB-Map backend processing rate in Hz.",
        ),
        DeclareLaunchArgument(
            "odom_reset_countdown",
            default_value="0",
            description=(
                "Reinitialize RGB-D odometry after this many consecutive "
                "registration failures. Set to 0 to disable recovery."
            ),
        ),
        DeclareLaunchArgument(
            "grid_max_ground_height",
            default_value="0.15",
            description=(
                "Ground-height gate for the calibrated camera. Validate low obstacles."
            ),
        ),
        DeclareLaunchArgument(
            "grid_max_ground_angle",
            default_value="15.0",
            description=(
                "Maximum angle in degrees between a point normal and the ground "
                "normal for normal-based ground segmentation."
            ),
        ),
        DeclareLaunchArgument(
            "grid_flat_obstacles_detected",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Treat separate horizontal surfaces above the main ground plane "
                "as obstacles. Disabling this can also hide tables and shelves."
            ),
        ),
        DeclareLaunchArgument(
            "grid_max_obstacle_height",
            default_value="0.70",
            description="Ignore obstacle points above this height in meters.",
        ),
        DeclareLaunchArgument(
            "grid_prob_miss",
            default_value="0.35",
            description=(
                "Global occupancy miss probability. Lower values clear cells "
                "more strongly when later observations see free space."
            ),
        ),
        DeclareLaunchArgument("use_viz", default_value="false"),
        DeclareLaunchArgument("debug_checks", default_value="false"),

        DeclareLaunchArgument("camera_frame", default_value="oak_rgb_camera_optical_frame"),
        DeclareLaunchArgument("publish_camera_bridge_tf", default_value="false"),
    ]

    debug_print_env = ExecuteProcess(
        condition=IfCondition(debug_checks),
        cmd=[
            "bash", "-lc",
            (
                "echo '[DEBUG] ROS env:'; "
                "echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID; "
                "echo ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY; "
                "echo RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; "
                "echo CYCLONEDDS_URI=$CYCLONEDDS_URI; "
                "echo DISPLAY=$DISPLAY; "
                "echo '[DEBUG] Host IPs:'; "
                "hostname -I || true"
            )
        ],
        output="screen",
    )

    delayed_topic_check = TimerAction(
        condition=IfCondition(debug_checks),
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-lc",
                    (
                        "echo '[DEBUG] Topic list after 8s:'; "
                        "ros2 topic list | grep -E 'oak|rgbd|odom|rtabmap|tf|map' || true; "
                        "echo '[DEBUG] /rgbd_image info:'; "
                        "ros2 topic info /rgbd_image -v || true; "
                        "echo '[DEBUG] /odom info:'; "
                        "ros2 topic info /odom -v || true"
                    )
                ],
                output="screen",
            )
        ],
    )

    rgbd_odometry = TimerAction(
        condition=IfCondition(run_rgbd_odometry),
        period=2.0,
        actions=[
            LogInfo(msg="[rtab_debug] Starting rgbd_odometry after rgbd_sync delay..."),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "subscribe_rgbd": True,
                    "frame_id": base_frame,
                    "odom_frame_id": "rgbd_odom",
                    "publish_tf": False,
                    "wait_for_transform": ParameterValue(wait_for_transform, value_type=float),
                    "qos": 2,
                    "Reg/Force3DoF": "true",
                    "queue_size": 10,
                    #"wait_imu_to_init": True,

                    "Odom/ResetCountdown": ParameterValue(
                        odom_reset_countdown,
                        value_type=str,
                    ),
                    "Odom/ImageDecimation": ParameterValue(
                        odom_image_decimation,
                        value_type=str,
                    ),
                    "Odom/Strategy": "0",
                    "Odom/Holonomic": "false",
                    "Vis/MinInliers": "20",
                    "Vis/MaxFeatures": "1000",
                    "GFTT/MinDistance": "5",

                    "guess_frame_id": ParameterValue(
                        PythonExpression(
                            ["'odom' if '", use_odom_guess, "' == 'true' else ''"]
                        ),
                        value_type=str,
                    ),
                    "guess_min_translation": 0.0,
                    "guess_min_rotation": 0.0,
                }],
                remappings=[
                    ("/rgbd_image", "/rgbd_image/compressed"),
                    #("/imu", "/oak/imu/data"),
                    ("/odom", "/rgbd/odom"),
                ],
            )
        ],
    )

    rtabmap = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="[rtab_debug] Starting rtabmap after odometry delay..."),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "subscribe_rgbd": True,
                    "frame_id": base_frame,
                    # Subscribe to filtered odometry, preserving its covariance.
                    "odom_frame_id": "",
                    "map_frame_id": map_frame,
                    "publish_tf": ParameterValue(publish_tf, value_type=bool),
                    "database_path": database_path,
                    "Mem/IncrementalMemory": ParameterValue(
                        PythonExpression(
                            ["'false' if '", localization, "' == 'true' else 'true'"]
                        ),
                        value_type=str,
                    ),
                    "Mem/InitWMWithAllNodes": ParameterValue(
                        localization,
                        value_type=str,
                    ),
                    "delete_db_on_start": False,
                    "qos": 2,
                    "wait_for_transform": ParameterValue(
                        wait_for_transform, value_type=float),
                    "subscribe_odom_info": False,
                    "odom_sensor_sync": True,
                    "approx_sync": True,
                    "approx_sync_max_interval": 0.05,
                    "topic_queue_size": 10,
                    "sync_queue_size": 20,
                    "qos_odom": 1,
                    "Reg/Force3DoF": "true",
                    "RGBD/ForceOdom3DoF": "true",
                    "RGBD/NeighborLinkRefining": "true",
                    "RGBD/LoopClosureReextractFeatures": "true",
                    "RGBD/LocalBundleOnLoopClosure": "true",
                    "Optimizer/Robust": "false",
                    "Rtabmap/DetectionRate": ParameterValue(
                        backend_detection_rate,
                        value_type=str,
                    ),
                    "Vis/MinInliers": "20",
                    "Grid/CellSize": "0.05",
                    "Grid/RangeMax": "2.0",
                    "Grid/DepthDecimation": "1",
                    "Grid/NormalsSegmentation": "true",
                    "Grid/MaxGroundHeight": ParameterValue(
                        grid_max_ground_height,
                        value_type=str,
                    ),
                    "Grid/MaxGroundAngle": ParameterValue(
                        grid_max_ground_angle,
                        value_type=str,
                    ),
                    "Grid/FlatObstacleDetected": ParameterValue(
                        grid_flat_obstacles_detected,
                        value_type=str,
                    ),
                    "Grid/MaxObstacleHeight": ParameterValue(
                        grid_max_obstacle_height,
                        value_type=str,
                    ),
                    "Grid/NoiseFilteringRadius": "0.08",
                    "Grid/NoiseFilteringMinNeighbors": "5",
                    "Grid/RayTracing": "true",
                    "GridGlobal/ProbMiss": ParameterValue(
                        grid_prob_miss,
                        value_type=str,
                    ),
                    "RGBD/OptimizeMaxError": "3.0",
                }],
                remappings=[
                    ("/rgbd_image", "/rgbd_image/compressed"),
                    ("/odom", odom_topic),
                ],
            )
        ],
    )

    rtabmap_viz = TimerAction(
        condition=IfCondition(use_viz),
        period=7.0,
        actions=[
            LogInfo(msg="[rtab_debug] Starting rtabmap_viz GUI..."),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "subscribe_rgbd": True,
                    "frame_id": base_frame,
                    "odom_frame_id": odom_frame,
                    "map_frame_id": map_frame,
                    "qos": 2,
                    "wait_for_transform": ParameterValue(
                        wait_for_transform, value_type=float),
                }],
                remappings=[
                    ("/rgbd_image", "/rgbd_image/compressed"),
                    ("/odom", odom_topic),
                ],
            )
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg="[rtab_debug] ===== Mondy RTAB debug launch starting ====="),
        LogInfo(msg=["[rtab_debug] Frames: map=", map_frame, " odom=", odom_frame, " base=", base_frame, " oak=", oak_root_frame]),

        debug_print_env,
        rgbd_odometry,
        rtabmap,
        rtabmap_viz,
        delayed_topic_check,
    ])
