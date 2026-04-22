# final_oak_d — Autonomous Object-Following for Pupper

An **independent ROS 2 node** that detects chairs and balls with an OAK-D Lite camera and publishes velocity commands on `/cmd_vel`. The Pupper's **default walking controller** (the pretrained policy that ships with the robot and is activated by the "X" button on the gamepad) subscribes to `/cmd_vel` and turns those commands into leg motions — so the only thing this package has to do is publish `Twist` messages.

This package is self-contained. It does not modify, extend, or depend on any other code on the robot.

## Architecture

This package owns everything above `/cmd_vel`. Everything below it is the robot's pre-installed walking stack, which we do not touch.

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
object_follower_node.py  (ROS 2 node)
    |
    | publishes to
    v
/cmd_vel  (geometry_msgs/Twist)
    :
    : (consumed by the Pupper's default walking controller,
    :  which is already running on the robot)
    v
Pupper legs
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

After `test_local.py` looks correct, copy the folder to the robot (e.g. via `scp`):

```bash
cd final_oak_d
python3 deploy.py
```

This runs `object_follower_node.py` and starts publishing Twist messages on `/cmd_vel`. The Pupper's default walking controller — the pretrained policy that's already running on the robot — subscribes to `/cmd_vel` and drives the legs accordingly, so the robot will walk toward whatever chair or ball the camera sees. No configuration change on the robot is required; we're just publishing on a topic the default stack already listens to.

The robot stops when the target is within `target_distance` (default 0.4 m) or when the target is not detected for `timeout_frames` consecutive frames.

## Coordinate Frames

DepthAI publishes detections in **camera frame**: `+x right, +y down, +z forward`. `follower.py` converts these to robot velocity commands:

- `bearing = atan2(-x_cam, z_cam)` — horizontal angle to object (positive = right)
- `distance = z_cam` — forward distance (approximation; ignores vertical offset)

Velocity output:

- `linear.x = k_forward * max(0, distance - target_distance)` — walk forward until close enough
- `angular.z = -k_yaw * bearing` — turn toward the object
- `linear.y = 0` — we don't strafe

When the target is reached or no detection is received for a few frames, the follower publishes a zero Twist so the Pupper stops.
