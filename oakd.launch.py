"""
Launches our two OAK-D nodes:
  - object_follower_node.py   (OAK-D + YOLO + cmd_vel publisher)
  - mission_controller_node.py (state machine + path-clear check)

The upstream Pupper v3 neural_controller stack must already be running
(`ros2 launch neural_controller launch.py` from the monorepo). Our nodes
only publish /person_following_cmd_vel — they do not drive the legs.
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess

SCRIPT_DIR = Path(__file__).parent
CONFIG_PATH = SCRIPT_DIR / "config.yaml"


def generate_launch_description():
    follower = ExecuteProcess(
        cmd=[
            sys.executable,
            str(SCRIPT_DIR / "object_follower_node.py"),
            "--ros-args",
            "--params-file",
            str(CONFIG_PATH),
        ],
        output="screen",
    )
    mission = ExecuteProcess(
        cmd=[
            sys.executable,
            str(SCRIPT_DIR / "mission_controller_node.py"),
            "--ros-args",
            "--params-file",
            str(CONFIG_PATH),
        ],
        output="screen",
    )
    return LaunchDescription([follower, mission])
