"""
Bridge selected ROS topics out to the remote teleoperation messaging bus.
"""

from __future__ import annotations

import base64
import json
import os
from functools import partial
from typing import Any, Dict, Tuple

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from rosidl_runtime_py import convert
from sensor_msgs.msg import Image, Imu
import yaml
import zmq


class TeleopBridge:
    """Minimal ZMQ publisher that sends JSON blobs."""

    def __init__(self, endpoint: str = "tcp://*:5556"):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SNDHWM, 10_000)
        self._sock.bind(endpoint)  # bind so SUB clients can connect

    def publish(self, topic: str, payload: Dict[str, Any]) -> None:
        message = json.dumps({"topic": topic, "data": payload})
        try:
            self._sock.send_string(message, flags=zmq.NOBLOCK)
            print(f"Published to {topic} ({len(message)} bytes)")
        except zmq.Again:
            # Drop if downstream is not ready.
            pass

    def close(self) -> None:
        try:
            self._sock.close(0)
        except Exception:
            pass


class TeleopNode(Node):
    """Bridge ROS topics out to the teleoperation bridge bus."""

    # topic -> (message type, is_image)
    _BRIDGE_TOPICS: Dict[str, Tuple[type, bool]] = {
        "/imu/data": (Imu, False),
        "/main_camera/image": (Image, True),
    }

    def __init__(self) -> None:
        super().__init__("teleop_node")

        default_config = self._default_config_path()
        self.declare_parameter("config_file", default_config)
        config_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        if not config_path:
            config_path = default_config

        config = self._load_config(config_path)
        endpoint = self._resolve_endpoint(config)

        self.declare_parameter("teleop_bridge_endpoint", endpoint)
        endpoint_param = (
            self.get_parameter("teleop_bridge_endpoint")
            .get_parameter_value()
            .string_value
        )
        if endpoint_param:
            endpoint = endpoint_param

        self._bridge = TeleopBridge(endpoint)
        # Keep strong references to prevent subscriptions from being garbage collected.
        self._subscriptions: list = []

        for topic, (msg_type, is_image) in self._BRIDGE_TOPICS.items():
            callback = partial(self._handle_message, topic, is_image)
            self._subscriptions.append(
                self.create_subscription(msg_type, topic, callback, 10)
            )

        self.get_logger().info(
            f"Teleop bridge publishing to {endpoint} from "
            f"{', '.join(self._BRIDGE_TOPICS.keys())} "
            f"(config: {config_path or 'built-in defaults'})"
        )

    def destroy_node(self) -> None:
        self._bridge.close()
        return super().destroy_node()

    def _default_config_path(self) -> str:
        """Return the packaged config path if available, else fall back to source tree."""
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
        """Load YAML config into a dictionary; invalid input falls back to {}."""
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

    def _resolve_endpoint(self, config: Dict[str, Any]) -> str:
        """Compute the ZMQ endpoint string using config overrides and sensible defaults."""
        bridge_cfg = config.get("teleop_bridge") or config.get("sensor_bus") or {}
        if not isinstance(bridge_cfg, dict):
            bridge_cfg = {}

        endpoint = bridge_cfg.get("endpoint")
        if isinstance(endpoint, str) and endpoint:
            return endpoint

        host = bridge_cfg.get("host", "*")  # default bind-all
        port = bridge_cfg.get("port", 5556)
        return f"tcp://{host}:{port}"

    def _handle_message(self, topic: str, is_image: bool, msg) -> None:
        """Convert a ROS message to JSON-serialisable payload and push it onto the bus."""
        payload = convert.message_to_ordereddict(msg)

        if is_image:
            data = payload.get("data", [])
            if isinstance(data, list):
                payload["data"] = base64.b64encode(bytes(data)).decode("ascii")

        try:
            self._bridge.publish(topic, payload)
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to publish {topic} payload to teleop bridge: {exc}"
            )


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
