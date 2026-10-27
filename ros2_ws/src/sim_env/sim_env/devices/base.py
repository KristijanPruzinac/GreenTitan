"""Common plumbing for device-model nodes.

Every device node takes one string parameter ``config`` (JSON) holding its YAML section, so a
device is described entirely in the robot file. The launch file also injects scene facts
(``scene``: origin, topics) into the same dict.
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class DeviceNode(Node):
    def __init__(self, default_name):
        super().__init__(default_name)
        self.cfg = json.loads(self.declare_parameter('config', '{}').value)
        self.scene = self.cfg.get('scene', {})
        self.rng = np.random.default_rng(self.cfg.get('seed'))

    def c(self, key, default=None):
        node = self.cfg
        for part in key.split('.'):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


class GaussMarkov:
    """First-order Gauss-Markov process (bias random walk with correlation time ``tau``)."""

    def __init__(self, rng, sigma, tau, dims=1, initial_sigma=None):
        self.rng, self.sigma, self.tau = rng, float(sigma), float(tau)
        self.x = rng.normal(0.0, self.sigma if initial_sigma is None else initial_sigma, dims)

    def step(self, dt):
        if self.tau <= 0 or self.sigma <= 0 or dt <= 0:
            return self.x
        a = np.exp(-dt / self.tau)
        self.x = a * self.x + self.rng.normal(0.0, self.sigma * np.sqrt(1 - a * a), self.x.shape)
        return self.x


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def spin(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
