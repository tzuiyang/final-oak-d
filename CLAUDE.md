# Project Goal

Final project for the Pupper robot (Pupper v3 hardware, running the stack in
`reference/pupperv3-monorepo/`). This repo adds an OAK-D based vision and
follow-behavior layer on top of that stack.

## User-facing behavior

1. **Stand still and identify.** Pupper stands in place while the OAK-D runs
   YOLO inference. All detected objects (from the supported COCO classes) are
   exposed to the user, along with their class and 3D position in the camera
   frame.
2. **User selects a target.** The user picks one of the identified objects
   as the goal (UI / CLI / service call — TBD).
3. **Path check.** Before moving, Pupper checks whether the path to the
   selected object is clear (using OAK-D depth and/or other obstacle info
   from the Pupper stack).
4. **Act on the result:**
   - **Clear path:** Pupper walks toward the selected object using the
     existing follower logic (publishes `/cmd_vel`, which the Pupper v3
     `neural_controller` consumes).
   - **Blocked path:** Pupper does **not** move and surfaces an error to
     the user.

## Current code state

- `detector.py` — OAK-D + YOLO spatial detection pipeline (depthai v2).
- `follower.py` — reactive P-controller turning a detection into a
  `(x_vel, y_vel, ang_vel)` command.
- `object_follower_node.py` — owns the OAK-D, publishes
  `/oakd/detections` every frame, subscribes to `/oakd/target`, and only
  publishes `/person_following_cmd_vel` while engaged.
- `mission_controller_node.py` — state machine: prints visible objects,
  accepts user selection on `/oakd/select_target`, runs the path-clear
  check, then engages the follower or publishes on `/oakd/error`.
- `web_ui_node.py` — Flask server on port 8080. Shows the annotated
  camera stream (MJPEG) and forwards click coordinates to
  `/oakd/select_target`. Clicks on empty space disengage.
- `templates/index.html` — the UI page served by `web_ui_node.py`.
- `oakd.launch.py` — launches the three nodes above.
- `deploy.py` — downloads the YOLO blob if missing, then runs
  `ros2 launch oakd.launch.py`. Assumes the upstream stack is already up.
- `reference/pupperv3-monorepo/` — upstream Pupper v3 control code,
  cloned shallow and gitignored.

## Topic wiring

                                 (OAK-D hardware)
                                        │
                                        ▼
    ┌──────────────── object_follower_node ──────────────┐
    │                                                    │
    │  /oakd/detections (JSON)        /oakd/frame_jpeg   │
    │  /person_following_cmd_vel      (CompressedImage)  │
    │                                                    │
    └────────────▲───────────────────────────────────────┘
                 │                │            │
          /oakd/target             │            │
                 │                 ▼            ▼
        mission_controller_node   web_ui_node (Flask :8080)
                 ▲                  │
                 │                  │
       /oakd/select_target ◀────────┘  (from browser clicks)

    Errors: mission_controller_node ──/oakd/error──▶ web_ui (red pill) + log
    Motion: object_follower  ──/person_following_cmd_vel──▶ upstream cmd_vel_mux

### User interaction

Browser: open `http://<pi-ip>:8080/`, click an object (green box = engaged,
amber = detected). Click empty space or the "stop" button to disengage.

CLI (works alongside the UI):
```bash
ros2 topic pub -1 /oakd/select_target std_msgs/String "{data: 'chair'}"
ros2 topic pub -1 /oakd/select_target std_msgs/String "{data: ''}"   # disengage
ros2 topic echo /oakd/error
```

## Gaps remaining before the full demo

- **Path-clear check is limited to YOLO-visible obstacles.** Walls, floor
  edges, and any object outside the COCO classes we act on (currently
  sports ball and chair) are invisible to the current check. Improving it
  requires streaming a full depth map from the OAK-D (add an `XLinkOut`
  for `stereo.depth`) and sampling a central ROI.
- **No richer error surface.** Errors go to stderr and `/oakd/error` only.
  If the team wants TTS / LED, wire it in the mission controller.
- **End-to-end hardware test on the robot** — syntax is checked; behavior
  is not. Needs a physical run.

## Minimum nodes to launch the demo

ROS distro: **Jazzy**. Build with `ros2_ws/build.sh` inside the monorepo.

### Upstream nodes we MUST launch (from `reference/pupperv3-monorepo/`)

Source: `ros2_ws/src/neural_controller/launch/launch.py`. These are the
strict minimum needed to make the legs respond to `/cmd_vel`:

1. `robot_state_publisher` — publishes the URDF as `robot_description`.
2. `controller_manager/ros2_control_node` — the hardware interface.
3. `joint_state_broadcaster` spawner — publishes `/joint_states`.
4. `imu_sensor_broadcaster` spawner — IMU feedback the policy needs.
5. `neural_controller` spawner — the learned locomotion policy; this is
   the node that actually subscribes to the muxed `/cmd_vel` and drives
   the legs.
6. `cmd_vel_mux_node` — multiplexes cmd_vel sources. Inputs are
   hardcoded in `cmd_vel_mux_node.cpp:21-23`:
   `/teleop_cmd_vel` (prio 0, highest) > `/llm_cmd_vel` > `/person_following_cmd_vel`.
   **Our follower should publish to `/person_following_cmd_vel`** so
   the gamepad E-stop / teleop can always override us for safety.

### Upstream nodes we SHOULD launch (safety / UX)

7. `joy_linux_node` + `joy_utils/estop_controller` — gamepad E-stop on the
   L1 button. Drop only if you have a physical kill switch.
8. `teleop_twist_joy_node` — gamepad → `/teleop_cmd_vel` for manual recovery.

### Upstream nodes we can SKIP

- `camera_node` (Pi camera) — we use the OAK-D instead.
- `hailo_detection_node` — we do detection on the OAK-D instead.
- `person_following_node` — we replace this behavior.
- `foxglove_bridge` — visualization only; enable if debugging.
- `bag_recorder_node` — not required for the demo.
- `animation_controller_py_node`, `joint_state_throttler` — animations.
- `imu_to_tf_node` — only needed if something else consumes TF.
- The `three_legged_robot_controller`, `forward_position_controller`,
  `forward_kp_controller`, `forward_kd_controller` spawners — alternative
  controllers that stay inactive anyway.

### Our nodes (this repo)

- **[object_follower_node.py](object_follower_node.py)** *(exists — needs refactor)*
  Currently owns both detection and velocity command in one loop and
  publishes continuously to `/cmd_vel`. For the demo we need to:
  - Remap its output to `/person_following_cmd_vel` (go through the mux).
  - Gate publishing behind an "engaged" flag so it can stand still.
  - Accept a selected target (class + rough position).
- **mission_controller_node.py** *(new — needs to be written)*
  The demo's state machine:
  1. Subscribe to detections, publish the current list (topic or service).
  2. Accept user selection (CLI / ROS service / topic).
  3. Run the path-clear check on OAK-D depth in front of the robot.
  4. Either engage the follower with the target, or publish an error.
- Optional: split detection out of `object_follower_node.py` into a
  separate `object_detector_node.py` that publishes a `Detections` topic
  the mission controller and follower both consume. Cleaner, but not
  required if we just add control hooks to the existing node.

### Gotchas from the upstream code

- `cmd_vel_mux` deadband is 0.05 — commands under that magnitude are
  suppressed (`config.yaml:176`).
- `cmd_vel_mux` joystick timeout is 500 ms — publish at ≥2 Hz or the mux
  stops forwarding (`config.yaml:177`).
- `neural_controller` has ~2 s init + fade-in before it honors velocity
  commands. During that window, publish zero Twist (or nothing).
- Teleop always preempts our commands (by design) — if the robot is
  ignoring us, check `/cmd_vel_mux/active_source`.
