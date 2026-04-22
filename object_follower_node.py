#!/usr/bin/env python3
"""
ROS 2 node that owns the OAK-D pipeline.

Publishes:
  - /oakd/detections           std_msgs/String (JSON)   every frame
  - /person_following_cmd_vel  geometry_msgs/Twist      only when engaged

Subscribes:
  - /oakd/target               std_msgs/String          class_name to follow, or
                                                        "" to disengage (stand still)

Feeds into the upstream cmd_vel_mux as the lowest-priority source
(`/person_following_cmd_vel`) so teleop can always preempt us.
"""

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from detector import YoloSpatialDetector
from follower import FollowerConfig, ObjectFollower


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

        self._engaged_target = ""  # empty = stand still, don't publish cmd_vel
        self._had_target_in_view = False  # tracks target-visible transitions

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self._det_pub = self.create_publisher(String, "/oakd/detections", 10)
        self._target_sub = self.create_subscription(
            String, "/oakd/target", self._on_target, 10
        )

        self._detector_ctx = YoloSpatialDetector(blob_path)
        self._detector = self._detector_ctx.__enter__()
        self._detection_stream = self._detector.detections()
        self.get_logger().info(
            f"OAK-D pipeline started (model={blob_path}, cmd_vel={cmd_topic})"
        )

        self._timer = self.create_timer(0.01, self._tick)

    def _on_target(self, msg: String):
        new = (msg.data or "").strip()
        if new == self._engaged_target:
            return
        if new:
            self.get_logger().info(f"Engaging follower on target: {new}")
        else:
            self.get_logger().info("Disengaged — standing still")
        self._engaged_target = new
        self._had_target_in_view = False

    def _tick(self):
        detections = next(self._detection_stream, None)
        if detections is None:
            return

        self._publish_detections(detections)

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

        twist = Twist()
        twist.linear.x = cmd.x_vel
        twist.linear.y = cmd.y_vel
        twist.angular.z = cmd.ang_vel
        self._cmd_pub.publish(twist)

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
                }
                for d in detections
            ]
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._det_pub.publish(msg)

    def destroy_node(self):
        self._detector_ctx.__exit__(None, None, None)
        super().destroy_node()


def main():
    rclpy.init()
    node = ObjectFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
