from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_port = LaunchConfiguration("rosbridge_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rosbridge_port",
            default_value="9090",
            description="Port used by rosbridge websocket server",
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{"port": rosbridge_port}],
        ),
        Node(
            package="uav_fleet_gui",
            executable="uav_gui",
            name="uav_gui",
            output="screen",
        ),
    ])
