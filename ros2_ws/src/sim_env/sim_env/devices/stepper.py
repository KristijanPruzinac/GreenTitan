"""Stepper drive model: step rates (what a controller emits on STEP pins) -> joint speeds.

Covers the drive chain between a controller and the wheel: steps/rev, microstepping, gear
ratio, direction sign, the controller's own step-rate ceiling and an optional drive slip. Torque limits (stall) are on
the joint itself (``actuators.max_torque`` in the robot file).

Config::

    input_topic: hil/stepper_rate      # std_msgs/Float32MultiArray, steps/s, one entry per motor
    timeout: 0.5                       # s without input -> motors stop
    rate: 100                          # Hz joint command output
    motors:
      - {joint: left_joint, steps_per_rev: 200, microsteps: 1, gear_ratio: 10, sign: 1, max_step_rate: 500}

``slip`` (0..1, default 0) turns the wheel that much slower than its steps (a wheel slipping on grass,
a geared hub with play): one-sided slip curves a drive the controller thinks is straight.
"""
import math

from std_msgs.msg import Float32MultiArray, Float64

from .base import SENSOR_QOS, DeviceNode, spin


class Stepper(DeviceNode):
    def __init__(self):
        super().__init__('stepper_model')
        self.motors = self.c('motors', [])
        self.timeout = float(self.c('timeout', 0.5))
        self.rates = [0.0] * len(self.motors)
        self.last_rx = None
        topics = self.scene['joint_cmd_topics']
        self.pubs = [self.create_publisher(Float64, topics[m['joint']], 10) for m in self.motors]
        self.create_subscription(Float32MultiArray, self.c('input_topic', 'hil/stepper_rate'), self._on_rate, SENSOR_QOS)
        self.create_timer(1.0 / float(self.c('rate', 100)), self._tick)

    def _on_rate(self, msg):
        self.rates = [float(msg.data[i]) if i < len(msg.data) else 0.0 for i in range(len(self.motors))]
        self.last_rx = self.now_s()

    def _tick(self):
        alive = self.last_rx is not None and self.now_s() - self.last_rx <= self.timeout
        for m, rate, pub in zip(self.motors, self.rates, self.pubs):
            cap = float(m.get('max_step_rate', 0)) or math.inf
            steps = max(-cap, min(cap, rate)) if alive else 0.0
            per_rev = float(m.get('steps_per_rev', 200)) * float(m.get('microsteps', 1)) * float(m.get('gear_ratio', 1))
            slip = float(m.get('slip', 0.0))
            pub.publish(Float64(data=float(m.get('sign', 1)) * (1.0 - slip) * steps / per_rev * 2 * math.pi))


def main():
    spin(Stepper)
