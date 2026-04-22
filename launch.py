"""ROS 2 launch file for the object follower node."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

SCRIPT_DIR = Path(__file__).parent


def generate_launch_description():
    config_path = SCRIPT_DIR / "config.yaml"

    return LaunchDescription(
        [
            Node(
                package="final_oak_d",
                executable="object_follower_node.py",
                name="object_follower",
                output="screen",
                parameters=[str(config_path)],
            ),
        ]
    )
