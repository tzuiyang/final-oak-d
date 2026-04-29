#!/usr/bin/env python3
"""
Mission state machine for the OAK-D follow demo.

Flow:
  1. Pupper stands still. This node subscribes to /oakd/detections and
     periodically prints the visible objects so the operator can choose one.
  2. Operator selects a class by publishing on /oakd/select_target (or via
     the web UI, which publishes the same topic). Empty string disengages.
  3. While a target is requested, this node checks whether the forward
     corridor is clear of depth obstacles and other YOLO-visible detections
     closer than the target.
  4. A request is considered failed only after it has missed several
     consecutive evaluations (`miss_threshold`). Single-frame YOLO
     dropouts are ignored — the follower's own timeout handles those.
     Once a failure is declared, the follower is disengaged and an error
     is published on /oakd/error, but the request stays set so the node
     re-engages automatically if the target reappears.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__("mission_controller")

        self.declare_parameter("corridor_half_width_rad", 0.25)
        self.declare_parameter("obstacle_clearance_m", 0.2)
        self.declare_parameter("print_period_s", 2.0)
        self.declare_parameter("evaluate_period_s", 0.2)
        self.declare_parameter("require_depth_path_check", True)
        # How many consecutive evaluations without the target before we
        # surface an error and disengage. At the default 0.2 s eval period,
        # 8 misses ≈ 1.6 s of confirmed absence.
        self.declare_parameter("miss_threshold", 8)

        self._latest_detections: list = []
        self._latest_path_status: dict | None = None
        self._requested_target = ""
        self._engaged_target = ""
        self._miss_count = 0
        self._last_error = ""

        self.create_subscription(String, "/oakd/detections", self._on_detections, 10)
        self.create_subscription(String, "/oakd/path_clear", self._on_path_status, 10)
        self.create_subscription(String, "/oakd/select_target", self._on_select, 10)

        self._target_pub = self.create_publisher(String, "/oakd/target", 10)
        self._error_pub = self.create_publisher(String, "/oakd/error", 10)

        self.create_timer(
            self.get_parameter("print_period_s").value, self._print_standing_menu
        )
        self.create_timer(
            self.get_parameter("evaluate_period_s").value, self._evaluate
        )

        self.get_logger().info(
            "Mission controller ready. Select a target with: "
            "ros2 topic pub -1 /oakd/select_target std_msgs/String "
            "\"{data: 'chair'}\""
        )

    def _on_detections(self, msg: String):
        payload = json.loads(msg.data)
        self._latest_detections = payload.get("detections", [])

    def _on_path_status(self, msg: String):
        self._latest_path_status = json.loads(msg.data)

    def _on_select(self, msg: String):
        new = (msg.data or "").strip()
        if new == self._requested_target:
            return
        self._requested_target = new
        self._miss_count = 0
        if new:
            self.get_logger().info(f"User selected: {new}")
            # Clear any stale error from a previous selection.
            self._publish_error("")
        else:
            self.get_logger().info("User disengaged")
            self._publish_target("")
            self._publish_error("")

    def _print_standing_menu(self):
        if self._requested_target or self._engaged_target:
            return
        if not self._latest_detections:
            self.get_logger().info("Standing still — no objects detected.")
            return
        summary = ", ".join(
            f"{d['class_name']} @ {d['distance']:.2f}m"
            for d in self._latest_detections
        )
        self.get_logger().info(f"Standing still — visible: {summary}")

    def _evaluate(self):
        if not self._requested_target:
            return

        target_class = self._requested_target
        snapshot = list(self._latest_detections)
        matching = [d for d in snapshot if d["class_name"] == target_class]
        threshold = int(self.get_parameter("miss_threshold").value)

        if not matching:
            self._miss_count += 1
            if self._miss_count == threshold:
                # First time we cross the threshold — disengage and warn.
                self._publish_error(f"No {target_class} visible")
                self._publish_target("")
            # Keep the request; we'll re-engage if it reappears.
            return

        # Target is visible this tick.
        self._miss_count = 0
        target = min(matching, key=lambda d: d["distance"])

        corridor = self.get_parameter("corridor_half_width_rad").value
        clearance = self.get_parameter("obstacle_clearance_m").value

        depth_error = self._depth_path_error(target, clearance)
        if depth_error is not None:
            self._publish_error(depth_error)
            self._publish_target("")
            return

        blockers = [
            d
            for d in snapshot
            if d is not target
            and abs(d["bearing"] - target["bearing"]) <= corridor
            and d["z"] < target["z"] - clearance
        ]

        if blockers:
            names = ", ".join(sorted({b["class_name"] for b in blockers}))
            self._publish_error(f"Path to {target_class} blocked by: {names}")
            self._publish_target("")
            return

        # All clear — engage.
        if self._engaged_target != target_class:
            self.get_logger().info(
                f"Path clear. Engaging follower toward {target_class}."
            )
        self._publish_target(target_class)
        self._publish_error("")  # clear any stale error now that we're engaged

    def _depth_path_error(self, target: dict, clearance: float):
        status = self._latest_path_status
        require_depth = bool(self.get_parameter("require_depth_path_check").value)
        if not status:
            if require_depth:
                return "Depth path check unavailable"
            return None

        if not status.get("valid", False):
            if require_depth:
                reason = status.get("reason", "invalid depth path check")
                return f"Depth path check unavailable: {reason}"
            return None

        nearest = status.get("nearest_obstacle_m")
        if nearest is None:
            return None

        target_z = float(target.get("z", target.get("distance", 0.0)))
        nearest = float(nearest)
        if nearest < target_z - clearance:
            return f"Path to {target['class_name']} blocked by depth obstacle at {nearest:.2f}m"
        return None

    def _publish_target(self, class_name: str):
        if class_name == self._engaged_target:
            return
        self._engaged_target = class_name
        msg = String()
        msg.data = class_name
        self._target_pub.publish(msg)

    def _publish_error(self, text: str):
        if text == self._last_error:
            return
        self._last_error = text
        if text:
            self.get_logger().warn(text)
        msg = String()
        msg.data = text
        self._error_pub.publish(msg)


def main():
    rclpy.init()
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
