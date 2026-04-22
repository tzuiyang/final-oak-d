# final_oak_d — Autonomous Object-Following for Pupper

Detect chairs and balls with an OAK-D Lite camera, then drive the Pupper's Lab 5 RL walking policy toward them using published velocity commands. Reuses the trained neural controller unchanged — this module only adds perception and a high-level planner.

## Architecture

```
OAK-D Lite (USB)
    |
    | RGB + Stereo Depth + on-device YOLO inference
    v
detector.py  (YoloSpatialDetector class)
    |
    | Detection(class_name, x, y, z in meters, confidence)
    v
follower.py  (ObjectFollower class)
    |
    | Twist(linear.x, linear.y, angular.z)
    v
/cmd_vel  (ROS 2 topic)
    |
    v
Lab 5 neural_controller  (unchanged)
    |
    v
Pupper joints
```

## Files

| File | Purpose |
|---|---|
| `download_model.py` | Fetches the YOLOv8n COCO `.blob` file from Luxonis model zoo into `models/` |
| `detector.py` | DepthAI pipeline. Runs YOLO on the camera itself, returns detections with 3D coordinates |
| `follower.py` | Converts a detection to velocity commands. P-controller on bearing + forward distance |
| `test_local.py` | Standalone macOS test: opens camera window, shows detections with distance overlay |
| `object_follower_node.py` | ROS 2 node for deployment on Pupper. Publishes to `/cmd_vel` |
| `launch.py` | ROS 2 launch file — starts the follower node with params from config.yaml |
| `config.yaml` | Follower parameters (target class, distance threshold, velocity gains) |
| `deploy.py` | Deployment entry point on robot — launches the ROS 2 pipeline |
| `requirements.txt` | Python dependencies (depthai, opencv-python, numpy) |

## Target Classes

Only two COCO classes are acted on — detections of anything else are ignored.

| COCO ID | Name |
|---|---|
| 32 | sports ball |
| 56 | chair |

## Usage

### Which script runs where

Keeping track of which computer runs which script is important because `final_oak_d/` lives in two places once deployed: on your Mac for development/testing, and on the Pupper for the real run.

| Script | Where to run it | When |
|---|---|---|
| `download_model.py` | Your Mac **and** the Pupper (once each) | One-time setup — pulls the YOLO `.blob` into `models/` |
| `test_local.py` | Your Mac only | Development / debugging — plug OAK-D into Mac, no robot involved |
| `object_follower_node.py` | The Pupper only | At runtime — ROS 2 node that publishes `/cmd_vel` |
| `deploy.py` | The Pupper only | Entry point — downloads model (if missing) and launches the node |

### Local testing (macOS, no robot)

```bash
cd /Users/tzu-iyang/NYU_ROB_UY_2004_Daniel/Labs/lab5/final_oak_d
pip install -r requirements.txt
python3 download_model.py        # one-time, ~30 seconds
python3 test_local.py            # live output, Ctrl-C to stop
```

Once the `.blob` is downloaded, `test_local.py` should work as long as the OAK-D is plugged in.

### What to validate in `test_local.py` output

- Hold a chair ~1 m away, centered → `z ≈ 1.0`, `bearing ≈ 0`, `x_vel > 0`, `ang_vel ≈ 0`
- Move the chair to your right → `x > 0`, `bearing > 0`, and critically: **`ang_vel < 0`** (negative = turn right in Pupper convention)
- Move the chair to your left → `x < 0`, `bearing < 0`, `ang_vel > 0`
- Walk the chair toward the camera (distance shrinks below 0.4 m) → `x_vel` should drop to 0

If `ang_vel` has the wrong sign (the robot would turn *away* from the object), fix it in `follower.py` before deploying — this is the one bug that's easy to verify on the laptop and painful to debug on the robot.

### On the Pupper

After `test_local.py` looks correct, copy the folder to the robot (e.g. via `scp`), start the Lab 5 neural controller in one terminal, then in a second terminal:

```bash
cd final_oak_d
python3 deploy.py
```

The robot walks toward whatever chair or ball the camera sees, stopping when within `target_distance` (default 0.4 m).

## Coordinate Frames

DepthAI publishes detections in **camera frame**: `+x right, +y down, +z forward`. `follower.py` converts these to robot velocity commands:

- `bearing = atan2(-x_cam, z_cam)` — horizontal angle to object (positive = right)
- `distance = z_cam` — forward distance (approximation; ignores vertical offset)

Velocity output:

- `linear.x = k_forward * max(0, distance - target_distance)` — walk forward until close enough
- `angular.z = -k_yaw * bearing` — turn toward the object
- `linear.y = 0` — we don't strafe

When the target is reached or no detection is received for a few frames, the follower publishes a zero Twist so the Pupper stops.
