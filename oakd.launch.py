"""
Launches the three OAK-D-side nodes:
  - object_follower_node.py    (OAK-D + YOLO + depth ROI + cmd_vel + frames)
  - mission_controller_node.py (state machine + path-clear check)
  - web_ui_node.py             (Flask server for click-to-select)

The upstream Pupper v3 control stack must already be running. Prefer launching
through `deploy.py`, which starts `pupper_minimal.launch.py` first; if launching
manually, run `pupper_minimal.launch.py` instead of the stock upstream
`neural_controller/launch.py` so the upstream person follower does not also
publish to /person_following_cmd_vel. Our nodes only publish
/person_following_cmd_vel — they do not drive the legs directly.
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
