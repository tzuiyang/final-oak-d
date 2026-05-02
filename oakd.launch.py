"""
Launches the OAK-D-side nodes plus a remapped teleop_twist_joy:

  - object_follower_node.py    (OAK-D + YOLO + cmd_vel mux + frames)
  - mission_controller_node.py (target-selection state machine)
  - web_ui_node.py             (Flask server for click-to-select)
  - teleop_twist_joy           (gamepad sticks -> /teleop_cmd_vel)

object_follower_node owns /cmd_vel and acts as the mux: in AUTO mode it
publishes follower output, in MANUAL mode it forwards /teleop_cmd_vel.
Toggle with the gamepad button configured in config.yaml.

The upstream Pupper stack should be launched with teleop:=False (deploy.py
does this) so it doesn't also publish to /cmd_vel and race our mux.
"""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

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
    # Mirror the upstream's teleop axis/scale config so manual feel matches
    # what the team is used to. Output goes to /teleop_cmd_vel; our follower
    # forwards that to /cmd_vel in MANUAL mode.
    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="oakd_teleop",
        output="screen",
        parameters=[{
            "axis_linear.x": 1,
            "axis_linear.y": 0,
            "axis_angular.yaw": 3,
            "scale_linear.x": 0.75,
            "scale_linear.y": 0.5,
            "scale_angular.yaw": 2.0,
            "require_enable_button": False,
        }],
        remappings=[("cmd_vel", "teleop_cmd_vel")],
    )

    return LaunchDescription(
        [
            _node("object_follower_node.py"),
            _node("mission_controller_node.py"),
            _node("web_ui_node.py"),
            teleop_twist_joy,
        ]
    )
