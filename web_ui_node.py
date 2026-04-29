#!/usr/bin/env python3
"""
Web UI for the OAK-D demo.

Runs a small Flask server on the Pi. The operator opens it from any laptop
on the network, sees the annotated camera stream, and clicks an object to
select it as the target. Clicks on empty space disengage.

Endpoints:
  GET  /              HTML page
  GET  /stream.mjpg   MJPEG stream of /oakd/frame_jpeg
  POST /select {x,y}  Normalized click coords (0..1). If a detection contains
                      the point, its class is published on /oakd/select_target.
                      Otherwise the current target is cleared (disengage).
  POST /disengage     Clears the target.
  GET  /state         JSON: current detections + latest error.

ROS side:
  Subscribes: /oakd/frame_jpeg, /oakd/detections, /oakd/path_clear, /oakd/error
  Publishes:  /oakd/select_target  (forwarded to mission_controller_node)
"""

import json
import threading
import time
from typing import Optional

import rclpy
from flask import Flask, Response, jsonify, render_template, request
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class WebUINode(Node):
    def __init__(self):
        super().__init__("web_ui")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self._latest_jpeg: Optional[bytes] = None
        self._frame_id = 0
        self._latest_detections: list = []
        self._latest_path_status: dict = {}
        self._latest_error: str = ""
        self._lock = threading.Lock()

        self.create_subscription(
            CompressedImage, "/oakd/frame_jpeg", self._on_frame, 10
        )
        self.create_subscription(
            String, "/oakd/detections", self._on_detections, 10
        )
        self.create_subscription(
            String, "/oakd/path_clear", self._on_path_status, 10
        )
        self.create_subscription(
            String, "/oakd/error", self._on_error, 10
        )
        self._select_pub = self.create_publisher(
            String, "/oakd/select_target", 10
        )

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self._app = _build_app(self)
        threading.Thread(
            target=self._app.run,
            kwargs={
                "host": host,
                "port": port,
                "debug": False,
                "use_reloader": False,
                "threaded": True,
            },
            daemon=True,
        ).start()
        self.get_logger().info(f"Web UI serving at http://{host}:{port}/")

    def _on_frame(self, msg: CompressedImage):
        with self._lock:
            self._latest_jpeg = bytes(msg.data)
            self._frame_id += 1

    def _on_detections(self, msg: String):
        payload = json.loads(msg.data)
        with self._lock:
            self._latest_detections = payload.get("detections", [])

    def _on_path_status(self, msg: String):
        payload = json.loads(msg.data)
        with self._lock:
            self._latest_path_status = payload

    def _on_error(self, msg: String):
        with self._lock:
            self._latest_error = msg.data

    def get_frame(self) -> tuple[Optional[bytes], int]:
        with self._lock:
            return self._latest_jpeg, self._frame_id

    def get_detections(self) -> list:
        with self._lock:
            return list(self._latest_detections)

    def get_error(self) -> str:
        with self._lock:
            return self._latest_error

    def get_path_status(self) -> dict:
        with self._lock:
            return dict(self._latest_path_status)

    def publish_target(self, class_name: str):
        msg = String()
        msg.data = class_name
        self._select_pub.publish(msg)


def _build_app(node: WebUINode) -> Flask:
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/stream.mjpg")
    def stream():
        return Response(
            _mjpeg_generator(node),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/select", methods=["POST"])
    def select():
        body = request.get_json(silent=True) or {}
        x = body.get("x")
        y = body.get("y")
        if x is None or y is None:
            return jsonify(ok=False, error="missing coordinates"), 400
        det = _pick_clicked(node.get_detections(), float(x), float(y))
        if det is None:
            node.publish_target("")
            return jsonify(ok=True, class_name="", disengaged=True)
        node.publish_target(det["class_name"])
        node.get_logger().info(
            f"UI click -> {det['class_name']} @ {det['distance']:.2f}m"
        )
        return jsonify(ok=True, class_name=det["class_name"])

    @app.route("/disengage", methods=["POST"])
    def disengage():
        node.publish_target("")
        return jsonify(ok=True)

    @app.route("/state")
    def state():
        return jsonify(
            detections=node.get_detections(),
            path_clear=node.get_path_status(),
            error=node.get_error(),
        )

    return app


def _mjpeg_generator(node: WebUINode):
    boundary = b"--frame\r\n"
    last_id = -1
    while True:
        frame, frame_id = node.get_frame()
        if frame is None or frame_id == last_id:
            time.sleep(0.03)
            continue
        last_id = frame_id
        yield (
            boundary
            + b"Content-Type: image/jpeg\r\nContent-Length: "
            + str(len(frame)).encode()
            + b"\r\n\r\n"
            + frame
            + b"\r\n"
        )


def _pick_clicked(detections, x: float, y: float):
    """Return the smallest (most-specific) detection whose bbox contains (x,y).

    x, y are normalized [0, 1] image coordinates (top-left origin).
    """
    hits = [
        d
        for d in detections
        if d["bbox_xmin"] <= x <= d["bbox_xmax"]
        and d["bbox_ymin"] <= y <= d["bbox_ymax"]
    ]
    if not hits:
        return None
    return min(
        hits,
        key=lambda d: (d["bbox_xmax"] - d["bbox_xmin"])
        * (d["bbox_ymax"] - d["bbox_ymin"]),
    )


def main():
    rclpy.init()
    node = WebUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
