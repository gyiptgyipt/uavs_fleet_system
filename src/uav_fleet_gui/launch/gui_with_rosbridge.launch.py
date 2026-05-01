from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_port = LaunchConfiguration("rosbridge_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rosbridge_port",
            default_value="9090",
            description="Port used by rosbridge websocket server",
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("rosbridge_server"),
                    "launch",
                    "rosbridge_websocket_launch.xml",
                ])
            ),
            launch_arguments={"port": rosbridge_port}.items(),
        ),
        Node(
            package="uav_fleet_gui",
            executable="uav_gui",
            name="uav_gui",
            output="screen",
        ),
    ])
