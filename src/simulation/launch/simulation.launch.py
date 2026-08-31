"""Launch the grout-line simulation for a chosen scenario."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="happy_path"),
            Node(
                package="simulation",
                executable="simulation",
                name="simulation",
                output="screen",
                parameters=[{"scenario": scenario}],
            ),
        ]
    )
