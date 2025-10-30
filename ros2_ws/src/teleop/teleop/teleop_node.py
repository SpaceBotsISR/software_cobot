#!/usr/bin/env python3
"""
Bridge selected ROS topics out to the remote teleoperation messaging bus,
and receive command topics (e.g., /space_cobot/cmd_vel) from the UI to ROS 2.
"""

from __future__ import annotations

import base64
import json
import os
from functools import partial
import time
from typing import Any, Dict, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from rosidl_runtime_py import convert
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, Twist
import yaml
import zmq


class TeleopBridge:
    """Two-socket ZMQ publisher: one for sensors, one for images."""

    def __init__(
        self,
        sensor_endpoint: str = "tcp://*:5556",
        image_endpoint: str = "tcp://*:5560",
    ):
        self._ctx = zmq.Context.instance()

        self._sensor_sock = self._ctx.socket(zmq.PUB)
        self._sensor_sock.setsockopt(zmq.LINGER, 0)
        self._sensor_sock.setsockopt(zmq.SNDHWM, 100)
        self._sensor_sock.bind(sensor_endpoint)

        self._image_sock = self._ctx.socket(zmq.PUB)
        self._image_sock.setsockopt(zmq.LINGER, 0)
        self._image_sock.setsockopt(zmq.SNDHWM, 3)
        self._image_sock.bind(image_endpoint)
        self._last_publish_time: float | None = None

    def publish(
        self, topic: str, payload: Dict[str, Any], is_image: bool = False
    ) -> None:
        message = json.dumps({"topic": topic, "data": payload})
        try:
            if is_image:
                self._image_sock.send_string(message, flags=zmq.NOBLOCK)
                now = time.monotonic()
                dt = (
                    now - self._last_publish_time
                    if self._last_publish_time is not None
                    else 0.0
                )
                self._last_publish_time = now
                print(f"Sent image message on topic: {topic} (dt={dt:.3f}s)")
            else:
                self._sensor_sock.send_string(message, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    def close(self) -> None:
        try:
            self._sensor_sock.close(0)
        except Exception:
            pass
        try:
            self._image_sock.close(0)
        except Exception:
            pass


class TeleopNode(Node):
    """Bridge ROS topics out and command topics in."""

    _BRIDGE_TOPICS: Dict[str, Tuple[type, bool]] = {
        "/imu/data": (Imu, False),
        "/space_cobot/pose": (PoseStamped, False),
        "/main_camera/image": (Image, True),
    }

    def __init__(self) -> None:
        super().__init__("teleop_node")

        default_config = self._default_config_path()
        self.declare_parameter("config_file", default_config)
        config_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        ) or default_config
        config = self._load_config(config_path)

        sensor_endpoint, image_endpoint = self._resolve_endpoints(config)
        self.declare_parameter("teleop_bridge_sensor_endpoint", sensor_endpoint)
        self.declare_parameter("teleop_bridge_image_endpoint", image_endpoint)
        sensor_param = (
            self.get_parameter("teleop_bridge_sensor_endpoint")
            .get_parameter_value()
            .string_value
        ) or sensor_endpoint
        image_param = (
            self.get_parameter("teleop_bridge_image_endpoint")
            .get_parameter_value()
            .string_value
        ) or image_endpoint

        self._bridge = TeleopBridge(sensor_param, image_param)

        # ROS publishers for inbound commands
        self._cmd_vel_pub = self.create_publisher(Twist, "/space_cobot/cmd_vel", 10)
        self._last_cmd_vel_time: float | None = None

        # ZMQ command intake (UI -> robot)
        self._cmd_ctx = zmq.Context.instance()
        self._cmd_sock = self._cmd_ctx.socket(zmq.SUB)
        self._cmd_sock.setsockopt(zmq.RCVHWM, 1000)
        self._cmd_sock.setsockopt(zmq.RCVTIMEO, 1)
        self._cmd_sock.setsockopt(zmq.LINGER, 0)
        self._cmd_sock.setsockopt(zmq.SUBSCRIBE, b"")

        cmd_endpoint = self._resolve_cmd_endpoint(config)
        self._cmd_sock.bind(cmd_endpoint)

        # Keep strong refs to subscriptions
        self._subscriptions: list = []
        for topic, (msg_type, is_image) in self._BRIDGE_TOPICS.items():
            callback = partial(self._handle_message, topic, is_image)
            self._subscriptions.append(
                self.create_subscription(msg_type, topic, callback, 10)
            )

        # Poll inbound command socket at 100 Hz
        self._cmd_timer = self.create_timer(0.01, self._poll_command_bus)

        self.get_logger().info(
            f"Teleop bridge PUB sensors on {sensor_param}; images on {image_param}; "
            f"commands SUB on {cmd_endpoint}; "
            f"topics out: {', '.join(self._BRIDGE_TOPICS.keys())} "
            f"(config: {config_path or 'built-in defaults'})"
        )

    def destroy_node(self) -> None:
        try:
            self._cmd_sock.close(0)
        except Exception:
            pass
        self._bridge.close()
        return super().destroy_node()

    def _default_config_path(self) -> str:
        try:
            share_dir = get_package_share_directory("teleop")
        except (PackageNotFoundError, FileNotFoundError):
            share_dir = None

        if share_dir:
            candidate = os.path.join(share_dir, "config", "teleop_bridge.yaml")
            if os.path.exists(candidate):
                return candidate

        local_candidate = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "..", "config", "teleop_bridge.yaml"
            )
        )
        return local_candidate

    def _load_config(self, path: str) -> Dict[str, Any]:
        if not path:
            return {}
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except FileNotFoundError:
            self.get_logger().warning(f"Config file not found: {path}")
            return {}
        except Exception as exc:
            self.get_logger().warning(f"Failed to read config file {path}: {exc}")
            return {}
        if not isinstance(data, dict):
            self.get_logger().warning(
                f"Config file {path} must define a dictionary at the top level"
            )
            return {}
        return data

    def _resolve_endpoints(self, config: Dict[str, Any]) -> Tuple[str, str]:
        bridge_cfg = config.get("teleop_bridge") or config.get("sensor_bus") or {}
        if not isinstance(bridge_cfg, dict):
            bridge_cfg = {}
        sensor_ep = bridge_cfg.get("sensor_endpoint")
        image_ep = bridge_cfg.get("image_endpoint")
        if (
            isinstance(sensor_ep, str)
            and sensor_ep
            and isinstance(image_ep, str)
            and image_ep
        ):
            return sensor_ep, image_ep
        host = bridge_cfg.get("host", "*")
        sensor_port = bridge_cfg.get("sensor_port", 5556)
        image_port = bridge_cfg.get("image_port", 5560)
        return f"tcp://{host}:{sensor_port}", f"tcp://{host}:{image_port}"

    def _resolve_cmd_endpoint(self, config: Dict[str, Any]) -> str:
        cmd_cfg = config.get("teleop_cmd") or config.get("command_bus") or {}
        if not isinstance(cmd_cfg, dict):
            cmd_cfg = {}
        endpoint = cmd_cfg.get("endpoint")
        if isinstance(endpoint, str) and endpoint:
            return endpoint
        host = cmd_cfg.get("host", "*")
        port = cmd_cfg.get("port", 5557)
        return f"tcp://{host}:{port}"

    def _handle_message(self, topic: str, is_image: bool, msg) -> None:
        payload = convert.message_to_ordereddict(msg)

        if is_image:
            try:
                h = int(payload["height"])
                w = int(payload["width"])
                enc = str(payload.get("encoding", "")).lower()
                raw = payload.get("data", [])
                if isinstance(raw, list):
                    buf = bytes(raw)
                elif isinstance(raw, str):
                    buf = base64.b64decode(raw)
                else:
                    buf = bytes()

                if enc in ("rgb8", "bgr8", "mono8", "rgba8", "bgra8"):
                    arr = np.frombuffer(buf, dtype=np.uint8)
                    if enc == "mono8":
                        img = arr.reshape(h, w)
                        ok, jpeg = cv2.imencode(
                            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                        )
                    else:
                        if "a" in enc:
                            channels = 4
                        else:
                            channels = 3
                        img = arr.reshape(h, w, channels)
                        if enc == "rgb8":
                            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        elif enc == "rgba8":
                            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                        elif enc == "bgra8":
                            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                        # bgr8 is already BGR
                        ok, jpeg = cv2.imencode(
                            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                        )

                    if ok:
                        payload = {
                            "width": w,
                            "height": h,
                            "encoding": "jpeg",
                            "data": base64.b64encode(jpeg.tobytes()).decode("ascii"),
                        }
                else:
                    # Unknown/raw format: try to treat as already-compressed
                    if not isinstance(raw, str):
                        payload["data"] = base64.b64encode(buf).decode("ascii")
                    payload["encoding"] = payload.get("encoding", "jpeg")
            except Exception:
                pass
        else:
            if isinstance(payload.get("data"), list):
                try:
                    payload["data"] = base64.b64encode(bytes(payload["data"])).decode(
                        "ascii"
                    )
                except Exception:
                    pass

        try:
            self._bridge.publish(topic, payload, is_image=is_image)
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to publish {topic} payload to teleop bridge: {exc}"
            )

    def _poll_command_bus(self) -> None:
        while True:
            try:
                raw = self._cmd_sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                obj = json.loads(raw.decode("utf-8"))
            except Exception:
                continue

            topic = obj.get("topic")
            data = obj.get("data")
            if topic == "/space_cobot/cmd_vel":
                twist = self._to_twist(data)
                if twist is not None:
                    now = time.monotonic()
                    dt = (
                        now - self._last_cmd_vel_time
                        if self._last_cmd_vel_time is not None
                        else 0.0
                    )
                    self._last_cmd_vel_time = now
                    if twist.linear.x > 0:
                        print(f"Received /space_cobot/cmd_vel (dt={dt:.3f}s)")
                    self._cmd_vel_pub.publish(twist)

    def _to_twist(self, data: Any) -> Twist | None:
        if not isinstance(data, dict):
            return None
        try:
            msg = convert.dictionary_to_message(Twist, data)
            return msg
        except Exception:
            pass
        try:
            msg = Twist()
            lin = data.get("linear", {})
            ang = data.get("angular", {})
            msg.linear.x = float(lin.get("x", 0.0))
            msg.linear.y = float(lin.get("y", 0.0))
            msg.linear.z = float(lin.get("z", 0.0))
            msg.angular.x = float(ang.get("x", 0.0))
            msg.angular.y = float(ang.get("y", 0.0))
            msg.angular.z = float(ang.get("z", 0.0))
            return msg
        except Exception:
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
