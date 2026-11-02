from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    model = LaunchConfiguration("model")
    rviz = LaunchConfiguration("rviz")
    jsp_gui = LaunchConfiguration("joint_state_gui")

    default_model = PathJoinSubstitution([
        FindPackageShare("mondy_description"),
        "urdf",
        "mondy.urdf.xacro",
    ])

    default_rviz = PathJoinSubstitution([
        FindPackageShare("mondy_description"),
        "rviz",
        "mondy.rviz",
    ])

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("rviz", default_value=default_rviz),
        DeclareLaunchArgument("joint_state_gui", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description
                }],
            remappings=[
        ("robot_description", "/mondy/robot_description"),
                ],
       ),

       # A somewhat non-ideal defualts publisher to keep tf alive even
       # when the physical rover is offline without overriding its joints
       Node(
           package="mondy_description",  # change if you put the node in another package
           executable="joint_state_defaults.py",
           name="joint_state_defualts",
           output="screen",
           parameters=[{
               "publish_rate_hz": 10.0,
               "stale_timeout_sec": 2.0,
               "joint_states_topic": "/joint_states",
               "corner_state_topic": "/corner_state",
               "drive_state_topic": "/drive_state",
               "own_stamp_cache_size": 200,
           }],
       ),

        # This is only for model testing. Replace with real /joint_states later.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(jsp_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz],
        ),
    ])
