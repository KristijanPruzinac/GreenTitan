"""Power model: charging-station contacts and a simple battery, what two ADC dividers would read.

Dock contacts: the world's ``station`` object (obstacles.py) has its contact point where the robot origin
(the axle midpoint) is when the plates touch. Contact = the origin is between 0 and ``contact_travel``
past it along the station axis, within ``lateral_tolerance`` of the axis and ``yaw_tolerance_deg`` of its
heading; then the contact voltage is the charger's ``charger_v``, else 0 (the station is dead off the dock).

Battery: state of charge drains with ``idle_w`` always, ``drive_w`` while the wheels turn and ``blade_w``
while the blade relay is on (third element of the stepper-rate input), and fills with ``charge_w`` while
docked; ``time_factor`` speeds that up for tests. Voltage linear in the charge between ``empty_v`` and
``full_v`` (a 7S pack, no curve).

Output: for the ESP32 HIL path, one block per message on ``output_topic`` (default ``hil/tof``: the
firmware's micro-ROS build has no subscription left, so the ToF topic carries it): byte 0x50, dock V and
pack V as float32 little endian. For tests also ``truth_topic`` (std_msgs/Float32MultiArray: contact 0/1,
along, lateral, yaw rel (rad), state of charge, dock V, pack V).

Config::

    rate: 10
    input_topic: hil/stepper_rate
    output_topic: hil/tof
    truth_topic: sim/power
    station: ''                  # name of the station object; '' = the first one in the world
    contact_travel: 0.08         # m the springs give
    lateral_tolerance: 0.03
    yaw_tolerance_deg: 10
    charger_v: 29.4
    battery: {capacity_wh: 250, soc: 0.9, empty_v: 22.0, full_v: 29.4, idle_w: 6, drive_w: 40, blade_w: 120,
              charge_w: 60, time_factor: 1.0}
"""
import math
import struct

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

from .. import obstacles
from .base import SENSOR_QOS, DeviceNode, quat_to_rot, spin

POWER_TAG = 0x50


class Power(DeviceNode):
    def __init__(self):
        super().__init__('power_model')
        name = self.c('station', '')
        objs = [o for o in (self.scene.get('world', {}).get('objects') or []) if o.get('type') == 'station'
                and (not name or o.get('name') == name)]
        self.station = obstacles.station_pose(objs[0]) if objs else None
        if self.station is None:
            self.get_logger().warn('no station object in the world: the contacts stay open')
        self.travel = float(self.c('contact_travel', 0.08))
        self.lat_tol = float(self.c('lateral_tolerance', 0.03))
        self.yaw_tol = math.radians(float(self.c('yaw_tolerance_deg', 10)))
        self.charger_v = float(self.c('charger_v', 29.4))
        b = self.c('battery', {}) or {}
        self.cap_j = float(b.get('capacity_wh', 250)) * 3600.0
        self.soc = float(b.get('soc', 0.9))
        self.empty_v, self.full_v = float(b.get('empty_v', 22.0)), float(b.get('full_v', 29.4))
        self.idle_w, self.drive_w = float(b.get('idle_w', 6)), float(b.get('drive_w', 40))
        self.blade_w, self.charge_w = float(b.get('blade_w', 120)), float(b.get('charge_w', 60))
        self.tf = float(b.get('time_factor', 1.0))
        self.pose = None
        self.rates = [0.0, 0.0, 0.0]
        self.last_t = None
        self.create_subscription(Odometry, self.scene['ground_truth_topic'], self._on_truth, 10)
        self.create_subscription(Float32MultiArray, self.c('input_topic', 'hil/stepper_rate'), self._on_rate, SENSOR_QOS)
        self.pub = self.create_publisher(UInt8MultiArray, self.c('output_topic', 'hil/tof'), SENSOR_QOS)
        tt = self.c('truth_topic', 'sim/power')
        self.truth_pub = self.create_publisher(Float32MultiArray, tt, 10) if tt else None
        self.create_timer(1.0 / float(self.c('rate', 10)), self._tick)

    def _on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        R = quat_to_rot((q.x, q.y, q.z, q.w))
        self.pose = (p.x, p.y, math.atan2(R[1, 0], R[0, 0]))

    def _on_rate(self, msg):
        self.rates = [float(v) for v in msg.data[:3]] + [0.0] * max(0, 3 - len(msg.data))

    def _contact(self):
        if self.pose is None or self.station is None:
            return False, 0.0, 0.0, 0.0
        sx, sy, syaw = self.station
        dx, dy = self.pose[0] - sx, self.pose[1] - sy
        c, s = math.cos(syaw), math.sin(syaw)
        along, lat = c * dx + s * dy, -s * dx + c * dy
        dyaw = (self.pose[2] - syaw + math.pi) % (2 * math.pi) - math.pi
        ok = 0.0 <= along <= self.travel and abs(lat) <= self.lat_tol and abs(dyaw) <= self.yaw_tol
        return ok, along, lat, dyaw

    def _tick(self):
        now = self.now_s()
        dt = 0.0 if self.last_t is None else max(0.0, min(1.0, now - self.last_t))
        self.last_t = now
        contact, along, lat, dyaw = self._contact()
        moving = abs(self.rates[0]) > 1.0 or abs(self.rates[1]) > 1.0
        p = self.idle_w + (self.drive_w if moving else 0.0) + (self.blade_w if self.rates[2] > 0.5 else 0.0)
        if contact:
            p -= self.charge_w
        self.soc = min(1.0, max(0.0, self.soc - p * dt * self.tf / self.cap_j))
        bat_v = self.empty_v + (self.full_v - self.empty_v) * self.soc
        dock_v = self.charger_v if contact else 0.0
        self.pub.publish(UInt8MultiArray(data=list(struct.pack('<Bff', POWER_TAG, dock_v, bat_v))))
        if self.truth_pub:
            self.truth_pub.publish(Float32MultiArray(data=[float(contact), along, lat, dyaw, self.soc, dock_v, bat_v]))


def main():
    spin(Power)
