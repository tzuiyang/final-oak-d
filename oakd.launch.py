"""
Launches the three OAK-D-side nodes:
  - object_follower_node.py    (OAK-D + YOLO + cmd_vel + frame publisher)
  - mission_controller_node.py (state machine + path-clear check)
  - web_ui_node.py             (Flask server for click-to-select)

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


def _node(script: str) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
            sys.executable,
            str(SCRIPT_DIR / script),
            "--ros-args",
            "--params-file",
            str(CONFIG_PATH),
        ],
        output="screen",
    )


def generate_launch_description():
    return LaunchDescription(
        [
            _node("object_follower_node.py"),
            _node("mission_controller_node.py"),
            _node("web_ui_node.py"),
        ]
    )
