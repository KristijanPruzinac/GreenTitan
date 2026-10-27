"""Differential-drive mixer: ``geometry_msgs/Twist`` -> wheel joint speeds (teleop, pure-ROS robots).

Wheel radius and track are taken from the robot's wheel definitions unless given.

Config::

    cmd_topic: cmd_vel
    left: [left]                 # wheel names from the robot file
    right: [right]
    wheel_radius: null           # override
    track: null                  # override (distance between left and right wheel centres)
    timeout: 0.5
"""
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from .base import DeviceNode, spin


class DiffDrive(DeviceNode):
    def __init__(self):
        super().__init__('diff_drive_mixer')
        wheels = self.scene['wheels']
        self.left, self.right = self.c('left', ['left']), self.c('right', ['right'])
        self.radius = float(self.c('wheel_radius') or wheels[self.left[0]]['radius'])
        self.track = float(self.c('track') or abs(wheels[self.left[0]]['position'][1] - wheels[self.right[0]]['position'][1]))
        self.timeout = float(self.c('timeout', 0.5))
        topics = self.scene['joint_cmd_topics']
        self.pubs = {n: self.create_publisher(Float64, topics[f'{n}_joint'], 10) for n in self.left + self.right}
        self.cmd = (0.0, 0.0)
        self.last_rx = None
        self.create_subscription(Twist, self.c('cmd_topic', 'cmd_vel'), self._on_cmd, 10)
        self.create_timer(0.02, self._tick)

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_rx = self.now_s()

    def _tick(self):
        v, w = self.cmd if self.last_rx is not None and self.now_s() - self.last_rx <= self.timeout else (0.0, 0.0)
        speeds = {'l': (v - w * self.track / 2) / self.radius, 'r': (v + w * self.track / 2) / self.radius}
        for n in self.left:
            self.pubs[n].publish(Float64(data=speeds['l']))
        for n in self.right:
            self.pubs[n].publish(Float64(data=speeds['r']))


def main():
    spin(DiffDrive)
