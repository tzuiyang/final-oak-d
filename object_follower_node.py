#!/usr/bin/env python3
"""
ROS 2 node that owns the OAK-D pipeline.

Publishes:
  - /oakd/detections           std_msgs/String (JSON)             every frame
  - /oakd/path_clear           std_msgs/String (JSON)             every frame
  - /oakd/frame_jpeg           sensor_msgs/CompressedImage (JPEG) ~15 Hz
  - /person_following_cmd_vel  geometry_msgs/Twist                only when engaged

Subscribes:
  - /oakd/target               std_msgs/String          class_name to follow, or
                                                        "" to disengage (stand still)

Feeds into the upstream cmd_vel_mux as the lowest-priority source
(`/person_following_cmd_vel`) so teleop can always preempt us.
"""

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import String


# Best-effort streaming QoS — drop stale messages instead of buffering them
# when subscribers fall behind. Must match the subscribers in mission_controller
# and web_ui or the connection is silently rejected.
_STREAMING_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

from detector import YoloSpatialDetector
from follower import FollowerConfig, ObjectFollower, VelocityCommand


FRAME_PUBLISH_PERIOD_S = 1.0 / 8.0  # 8 fps is plenty for the UI and halves WiFi load
JPEG_QUALITY = 50


class ObjectFollowerNode(Node):
    def __init__(self):
        super().__init__("object_follower")

        self.declare_parameter("model_blob_path", "models/yolov8n_coco_640x352.blob")
        self.declare_parameter("target_distance", 0.4)
        self.declare_parameter("k_forward", 0.8)
        self.declare_parameter("k_yaw", 1.2)
        self.declare_parameter("max_forward_vel", 0.5)
        self.declare_parameter("max_yaw_vel", 1.5)
        self.declare_parameter("timeout_frames", 10)
        self.declare_parameter("cmd_vel_topic", "/person_following_cmd_vel")
        self.declare_parameter("depth_corridor_width_ratio", 0.35)
        self.declare_parameter("depth_corridor_ymin", 0.35)
        self.declare_parameter("depth_corridor_ymax", 0.80)
        self.declare_parameter("min_valid_depth_m", 0.15)
        self.declare_parameter("max_valid_depth_m", 5.0)
        self.declare_parameter("depth_block_percentile", 10.0)
        self.declare_parameter("camera_flipped", False)
        # Throttle cmd_vel publishing. The walking policy needs steady input
        # to maintain a gait (it twitches in place if the input is too
        # sparse), so keep this near 10 Hz. Set to 0 to publish on every
        # detection.
        self.declare_parameter("command_period_s", 0.1)
        # Index into /joy.buttons for the AUTO/MANUAL toggle. Default 2 is
        # Square on a PS5 DualSense. Avoid 0,1 (gait switch),
        # 9 (activate), 12 (E-stop).
        self.declare_parameter("mode_toggle_button", 2)

        cfg = FollowerConfig(
            target_distance=self.get_parameter("target_distance").value,
            k_forward=self.get_parameter("k_forward").value,
            k_yaw=self.get_parameter("k_yaw").value,
            max_forward_vel=self.get_parameter("max_forward_vel").value,
            max_yaw_vel=self.get_parameter("max_yaw_vel").value,
            preferred_class=None,  # selection comes from /oakd/target
            timeout_frames=self.get_parameter("timeout_frames").value,
        )
        self._follower = ObjectFollower(cfg)

        blob_path = Path(self.get_parameter("model_blob_path").value)
        if not blob_path.is_absolute():
            blob_path = Path(__file__).parent / blob_path

        self._engaged_target = ""
        self._had_target_in_view = False
        self._last_frame_pub = 0.0
        self._last_cmd_pub = 0.0
        # AUTO = follower drives /cmd_vel from detections. MANUAL = forward
        # /teleop_cmd_vel straight through. Toggle with the gamepad button.
        self._manual_mode = False
        self._prev_toggle_button = False

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        # Streaming topics use BEST_EFFORT so a slow subscriber drops stale
        # messages instead of letting them queue. RELIABLE QoS would buffer
        # on the publisher when the subscriber falls behind, which on weak
        # WiFi shows up as multi-second UI latency.
        self._det_pub = self.create_publisher(String, "/oakd/detections", _STREAMING_QOS)
        self._path_pub = self.create_publisher(String, "/oakd/path_clear", _STREAMING_QOS)
        self._frame_pub = self.create_publisher(
            CompressedImage, "/oakd/frame_jpeg", _STREAMING_QOS
        )
        # /oakd/mode is RELIABLE so the UI is guaranteed to see toggles
        # even if it just connected.
        self._mode_pub = self.create_publisher(String, "/oakd/mode", 10)
        self._target_sub = self.create_subscription(
            String, "/oakd/target", self._on_target, 10
        )
        # The detection-processing _tick is the slow callback (YOLO consumption
        # + JPEG encoding can take 30-80 ms). Put it in its own mutually-
        # exclusive group, and put the teleop/joy subscriptions in a
        # reentrant group so they can run on a separate thread while _tick
        # is busy. Without this, MANUAL-mode stick movements visibly lag
        # because /teleop_cmd_vel callbacks queue behind _tick.
        self._tick_cb_group = MutuallyExclusiveCallbackGroup()
        self._fast_cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            Joy, "/joy", self._on_joy, 10, callback_group=self._fast_cb_group
        )
        self.create_subscription(
            Twist, "/teleop_cmd_vel", self._on_teleop_cmd, 10,
            callback_group=self._fast_cb_group,
        )

        flipped = bool(self.get_parameter("camera_flipped").value)
        self._detector_ctx = YoloSpatialDetector(blob_path, flipped=flipped)
        self._detector = self._detector_ctx.__enter__()
        self._detection_stream = self._detector.detections()
        self.get_logger().info(
            f"OAK-D pipeline started (model={blob_path}, cmd_vel={cmd_topic}, "
            f"flipped={flipped})"
        )

        self._timer = self.create_timer(
            0.01, self._tick, callback_group=self._tick_cb_group
        )
        # Publish the initial mode so the UI shows AUTO from the start.
        self._publish_mode()

    def _publish_mode(self):
        msg = String()
        msg.data = "MANUAL" if self._manual_mode else "AUTO"
        self._mode_pub.publish(msg)

    def _on_target(self, msg: String):
        new = (msg.data or "").strip()
        if new == self._engaged_target:
            return
        if new:
            self.get_logger().info(f"Engaging follower on target: {new}")
            # Reset the rate-limit clock so the first real velocity command
            # goes out immediately on the next tick — otherwise the robot
            # sits still for up to command_period_s after engagement.
            self._last_cmd_pub = 0.0
        else:
            self.get_logger().info("Disengaged — standing still")
            # Force-publish zero immediately, bypassing the rate limit.
            self._publish_velocity(VelocityCommand.zero(), force=True)
        self._engaged_target = new
        self._had_target_in_view = False

    def _on_joy(self, msg: Joy):
        btn_idx = int(self.get_parameter("mode_toggle_button").value)
        if btn_idx >= len(msg.buttons):
            return
        pressed = msg.buttons[btn_idx] == 1
        # Edge-trigger only — toggle on the press, ignore while held.
        if pressed and not self._prev_toggle_button:
            self._manual_mode = not self._manual_mode
            mode = "MANUAL" if self._manual_mode else "AUTO"
            self.get_logger().info(f"Mode -> {mode}")
            self._publish_mode()
            # Stop momentum during the transition.
            self._publish_velocity(VelocityCommand.zero(), force=True)
        self._prev_toggle_button = pressed

    def _on_teleop_cmd(self, msg: Twist):
        # Only forward when in MANUAL mode. AUTO ignores teleop entirely.
        if self._manual_mode:
            self._cmd_pub.publish(msg)

    def _tick(self):
        item = next(self._detection_stream, None)
        if item is None:
            return
        frame, depth_frame, detections = item

        self._publish_detections(detections)
        path_status = self._compute_path_status(depth_frame)
        self._publish_path_status(path_status)
        self._maybe_publish_frame(frame, detections, path_status)

        # In MANUAL the operator drives via /teleop_cmd_vel — _on_teleop_cmd
        # forwards those messages directly. Skip the autonomous follower.
        if self._manual_mode:
            return

        if not self._engaged_target:
            return

        filtered = [d for d in detections if d.class_name == self._engaged_target]
        cmd = self._follower.step(filtered)

        if filtered:
            best = min(filtered, key=lambda d: d.distance)
            self.get_logger().info(
                f"TARGET {best.class_name:12s}  conf={best.confidence:.2f}  "
                f"xyz=({best.x:+.2f},{best.y:+.2f},{best.z:+.2f})m  "
                f"dist={best.distance:.2f}m  bearing={best.bearing:+.3f}rad  "
                f"->  x_vel={cmd.x_vel:+.2f}  ang_vel={cmd.ang_vel:+.2f}"
            )
            self._had_target_in_view = True
        elif self._had_target_in_view:
            self.get_logger().info(
                f"{self._engaged_target} lost from view — publishing zero Twist"
            )
            self._had_target_in_view = False

        self._publish_velocity(cmd)

    def _publish_detections(self, detections):
        payload = {
            "detections": [
                {
                    "class_id": d.class_id,
                    "class_name": d.class_name,
                    "confidence": round(d.confidence, 3),
                    "x": round(d.x, 3),
                    "y": round(d.y, 3),
                    "z": round(d.z, 3),
                    "distance": round(d.distance, 3),
                    "bearing": round(d.bearing, 3),
                    "bbox_xmin": round(d.bbox_xmin, 4),
                    "bbox_ymin": round(d.bbox_ymin, 4),
                    "bbox_xmax": round(d.bbox_xmax, 4),
                    "bbox_ymax": round(d.bbox_ymax, 4),
                }
                for d in detections
            ]
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._det_pub.publish(msg)

    def _compute_path_status(self, depth_frame):
        width_ratio = _clamp_float(
            self.get_parameter("depth_corridor_width_ratio").value, 0.05, 1.0
        )
        y_min_ratio = _clamp_float(self.get_parameter("depth_corridor_ymin").value, 0.0, 1.0)
        y_max_ratio = _clamp_float(self.get_parameter("depth_corridor_ymax").value, 0.0, 1.0)
        if y_max_ratio <= y_min_ratio:
            y_min_ratio, y_max_ratio = 0.35, 0.80

        status = {
            "valid": False,
            "nearest_obstacle_m": None,
            "min_depth_m": None,
            "sample_count": 0,
            "roi": {
                "xmin": round((1.0 - width_ratio) / 2.0, 4),
                "xmax": round((1.0 + width_ratio) / 2.0, 4),
                "ymin": round(y_min_ratio, 4),
                "ymax": round(y_max_ratio, 4),
            },
            "depth_block_percentile": float(
                self.get_parameter("depth_block_percentile").value
            ),
        }
        if depth_frame is None:
            status["reason"] = "no depth frame"
            return status

        h, w = depth_frame.shape[:2]
        x1 = max(0, min(w - 1, int(status["roi"]["xmin"] * w)))
        x2 = max(x1 + 1, min(w, int(status["roi"]["xmax"] * w)))
        y1 = max(0, min(h - 1, int(y_min_ratio * h)))
        y2 = max(y1 + 1, min(h, int(y_max_ratio * h)))
        roi = depth_frame[y1:y2, x1:x2].astype(np.float32) / 1000.0

        min_depth = float(self.get_parameter("min_valid_depth_m").value)
        max_depth = float(self.get_parameter("max_valid_depth_m").value)
        valid = roi[
            np.isfinite(roi)
            & (roi >= min_depth)
            & (roi <= max_depth)
        ]
        status["sample_count"] = int(valid.size)
        if valid.size == 0:
            status["reason"] = "no valid depth samples"
            return status

        percentile = _clamp_float(
            self.get_parameter("depth_block_percentile").value, 0.0, 100.0
        )
        status["valid"] = True
        status["nearest_obstacle_m"] = round(float(np.percentile(valid, percentile)), 3)
        status["min_depth_m"] = round(float(np.min(valid)), 3)
        return status

    def _publish_path_status(self, status):
        msg = String()
        msg.data = json.dumps(status)
        self._path_pub.publish(msg)

    def _maybe_publish_frame(self, frame, detections, path_status):
        if frame is None:
            return
        now = time.monotonic()
        if now - self._last_frame_pub < FRAME_PUBLISH_PERIOD_S:
            return
        self._last_frame_pub = now

        annotated = _annotate(frame, detections, self._engaged_target, path_status)
        ok, jpeg = cv2.imencode(
            ".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if not ok:
            self.get_logger().warn("JPEG encode failed — dropping frame")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        self._frame_pub.publish(msg)

    def _publish_velocity(self, cmd: VelocityCommand, *, force: bool = False):
        if not force:
            period = float(self.get_parameter("command_period_s").value)
            now = time.monotonic()
            if now - self._last_cmd_pub < period:
                return
            self._last_cmd_pub = now
        else:
            self._last_cmd_pub = time.monotonic()
        twist = Twist()
        twist.linear.x = cmd.x_vel
        twist.linear.y = cmd.y_vel
        twist.angular.z = cmd.ang_vel
        self._cmd_pub.publish(twist)

    def destroy_node(self):
        self._detector_ctx.__exit__(None, None, None)
        super().destroy_node()


def _annotate(frame: np.ndarray, detections, engaged_target: str, path_status) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]
    _draw_path_roi(out, path_status)
    for d in detections:
        x1 = int(d.bbox_xmin * w)
        y1 = int(d.bbox_ymin * h)
        x2 = int(d.bbox_xmax * w)
        y2 = int(d.bbox_ymax * h)
        is_engaged = d.class_name == engaged_target
        color = (0, 200, 0) if is_engaged else (0, 200, 255)
        thickness = 3 if is_engaged else 2
        cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)
        label = (
            f"{d.class_name} {d.distance:.2f}m "
            f"{math.degrees(d.bearing):+.0f}deg"
        )
        _draw_label(out, label, (x1, y1), color)
    return out


def _draw_path_roi(img: np.ndarray, path_status):
    roi = (path_status or {}).get("roi") or {}
    try:
        h, w = img.shape[:2]
        x1 = int(float(roi["xmin"]) * w)
        x2 = int(float(roi["xmax"]) * w)
        y1 = int(float(roi["ymin"]) * h)
        y2 = int(float(roi["ymax"]) * h)
    except (KeyError, TypeError, ValueError):
        return
    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 180, 0), 1)


def _draw_label(img: np.ndarray, text: str, anchor, color):
    x, y = anchor
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    y_top = max(0, y - th - baseline - 4)
    cv2.rectangle(img, (x, y_top), (x + tw + 6, y_top + th + baseline + 4), color, -1)
    cv2.putText(
        img, text, (x + 3, y_top + th + 2),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA,
    )


def _clamp_float(value, low: float, high: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return low
    return max(low, min(high, parsed))


def main():
    rclpy.init()
    node = ObjectFollowerNode()
    # MultiThreadedExecutor so the slow _tick (YOLO + JPEG) doesn't block
    # the gamepad/teleop callbacks. 4 threads is plenty for this node.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
