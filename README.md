# final_oak_d - OAK-D Object Following for Pupper

This repo adds an OAK-D Lite vision and target-selection layer on top of the
Pupper v3 ROS 2 stack in `reference/pupperv3-monorepo/`.

The demo flow is:

1. Pupper stands still while the OAK-D runs YOLO spatial detection.
2. The operator selects a visible object from the web UI or `/oakd/select_target`.
3. `mission_controller_node.py` checks whether the OAK-D depth corridor is
   clear, with the old YOLO-visible blocker check kept as a secondary guard.
4. If clear, `object_follower_node.py` publishes commands to
   `/person_following_cmd_vel`, which the upstream Pupper `cmd_vel_mux` forwards
   to `/cmd_vel`.
5. If blocked, the robot stays stopped and the error is published on
   `/oakd/error`.

Teleop remains higher priority than our follower in the mux:
`/teleop_cmd_vel` > `/llm_cmd_vel` > `/person_following_cmd_vel`.

## Files

| File | Purpose |
|---|---|
| `detector.py` | DepthAI OAK-D YOLO spatial detection pipeline |
| `follower.py` | Converts a selected detection into Pupper velocity commands |
| `object_follower_node.py` | Owns OAK-D, publishes detections/frame JPEGs, and publishes follower commands only while engaged |
| `mission_controller_node.py` | Target-selection state machine and depth-corridor path-clear check |
| `web_ui_node.py` | Flask UI on port 8080 for camera stream, click-to-select, and errors |
| `templates/index.html` | Web UI page |
| `oakd.launch.py` | Launches this repo's three ROS nodes |
| `pupper_minimal.launch.py` | Launches only the upstream Pupper control nodes needed by this demo |
| `deploy.py` | One-command launcher for upstream Pupper control plus this OAK-D stack |
| `download_model.py` | Downloads the YOLO blob into `models/` |
| `test_local.py` | Non-ROS local OAK-D/follower smoke test |

## Usage On Pupper

Build the upstream workspace first:

```bash
cd reference/pupperv3-monorepo/ros2_ws
./build.sh
```

Then launch the demo:

```bash
cd /path/to/NYU_ROB_final_oak_d
python3 deploy.py
```

`deploy.py` uses `reference/pupperv3-monorepo/ros2_ws` by default when it
exists. Override with `PUPPER_WS=/path/to/ros2_ws` if needed.

Open the UI at:

```text
http://<pi-ip>:8080/
```

Click a detected object to request it as the target. Click empty space or the
stop button to disengage.

CLI selection works too:

```bash
ros2 topic pub -1 /oakd/select_target std_msgs/String "{data: 'chair'}"
ros2 topic pub -1 /oakd/select_target std_msgs/String "{data: ''}"
ros2 topic echo /oakd/error
```

## Local Checks

On a laptop with the OAK-D plugged in:

```bash
pip install -r requirements.txt
python3 download_model.py
python3 test_local.py
```

Expected follower signs:

| Target position | Expected command |
|---|---|
| Object centered and farther than `target_distance` | `x_vel > 0`, `ang_vel ~= 0` |
| Object to camera/right | `ang_vel < 0` |
| Object to camera/left | `ang_vel > 0` |
| Object closer than `target_distance` | `x_vel = 0` |

## Current Limitation

The path-clear check is a simple central ROI sampled from the OAK-D depth map.
It can catch non-COCO obstacles in front of the robot, but it does not build a
map, plan around objects, or reason about hazards outside the configured
corridor.
