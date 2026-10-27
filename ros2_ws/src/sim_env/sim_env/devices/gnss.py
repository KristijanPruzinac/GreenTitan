"""GNSS receiver model (RTK-capable).

Takes simulator ground truth and produces what a receiver reports: the *antenna* position (lever
arm, so chassis tilt shows up), solution states that switch fix/float/single/none as a Markov
chain, correlated (Gauss-Markov) position error plus white noise, Doppler-style velocity noise,
heading of motion with an accuracy that blows up at low speed, a fixed output rate and latency.

Outputs (each optional, empty topic disables it): ``sensor_msgs/NavSatFix``,
``geometry_msgs/TwistWithCovarianceStamped`` (ENU), and raw UBX NAV-PVT frames as
``std_msgs/UInt8MultiArray`` for feeding real firmware.

Config keys (all optional except where noted)::

    antenna_position: [x, y, z]       # robot frame, metres
    rate: 5.0                         # Hz
    latency: 0.05                     # s between epoch and output
    geoid_separation: 0.0             # ellipsoid minus MSL, metres
    initial_state: fix
    states:
      fix:    {h_sigma: 0.008, v_sigma: 0.015, bias_sigma: 0.006, bias_tau: 30, vel_sigma: 0.02, num_sv: 18}
      float:  {h_sigma: 0.05,  v_sigma: 0.10,  bias_sigma: 0.20,  bias_tau: 60, vel_sigma: 0.04, num_sv: 14}
      single: {h_sigma: 0.6,   v_sigma: 1.2,   bias_sigma: 1.0,   bias_tau: 300, vel_sigma: 0.08, num_sv: 10}
      none:   {}
    transitions:                      # mean dwell (s) and where to go when leaving
      fix:    {after: 900, to: {float: 1.0}}
      float:  {after: 20,  to: {fix: 0.9, single: 0.1}}
      single: {after: 30,  to: {float: 1.0}}
    zones:                            # areas that cap solution quality (trees, walls); the world
                                      # file's ``gnss_zones`` are added to these
      - {center: [5, 3], radius: 2.5, max_state: float}
    outputs: {fix_topic: gnss/fix, vel_topic: gnss/vel, ubx_topic: ''}
    frame_id: gnss
"""
import collections
import datetime
import math

import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8MultiArray

from .. import world
from . import ubx
from .base import SENSOR_QOS, DeviceNode, GaussMarkov, quat_to_rot, spin

QUALITY = ['none', 'single', 'float', 'fix']  # worst to best
CARR_SOLN = {'fix': 2, 'float': 1, 'single': 0, 'none': 0}
DEFAULT_STATES = {
    'fix': {'h_sigma': 0.008, 'v_sigma': 0.015, 'bias_sigma': 0.006, 'bias_tau': 30, 'vel_sigma': 0.02, 'num_sv': 18},
    'float': {'h_sigma': 0.05, 'v_sigma': 0.10, 'bias_sigma': 0.20, 'bias_tau': 60, 'vel_sigma': 0.04, 'num_sv': 14},
    'single': {'h_sigma': 0.6, 'v_sigma': 1.2, 'bias_sigma': 1.0, 'bias_tau': 300, 'vel_sigma': 0.08, 'num_sv': 10},
    'none': {'h_sigma': 50.0, 'v_sigma': 50.0, 'bias_sigma': 0.0, 'bias_tau': 0, 'vel_sigma': 5.0, 'num_sv': 0},
}
DEFAULT_TRANSITIONS = {
    'fix': {'after': 900, 'to': {'float': 1.0}},
    'float': {'after': 20, 'to': {'fix': 0.9, 'single': 0.1}},
    'single': {'after': 30, 'to': {'float': 1.0}},
}


class Gnss(DeviceNode):
    def __init__(self):
        super().__init__('gnss_model')
        self.antenna = np.array(self.c('antenna_position', [0.0, 0.0, 0.0]), float)
        self.latency = float(self.c('latency', 0.05))
        self.geoid = float(self.c('geoid_separation', 0.0))
        self.origin = self.scene['origin']
        self.states = {k: {**DEFAULT_STATES[k], **(self.c(f'states.{k}') or {})} for k in DEFAULT_STATES}
        self.transitions = self.c('transitions', DEFAULT_TRANSITIONS)
        self.zones = self.scene.get('gnss_zones', []) + self.c('zones', [])
        self.frame_id = self.c('frame_id', 'gnss')
        self.state = self.c('initial_state', 'fix')
        self.bias = self._new_bias(self.state)
        self.truth = None
        self.queue = collections.deque()
        self.last_epoch = None
        # UTC at clock time 0, so utc0 + now_s() is the epoch's UTC with sim time or wall time
        self.utc0 = datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(seconds=self.now_s())

        out = self.c('outputs', {})
        self.pub_fix = self._pub(NavSatFix, out.get('fix_topic', 'gnss/fix'))
        self.pub_vel = self._pub(TwistWithCovarianceStamped, out.get('vel_topic', 'gnss/vel'))
        self.pub_ubx = self._pub(UInt8MultiArray, out.get('ubx_topic', ''))
        self.create_subscription(Odometry, self.scene['ground_truth_topic'], self._on_truth, SENSOR_QOS)
        self.create_timer(1.0 / float(self.c('rate', 5.0)), self._epoch)
        self.create_timer(0.005, self._flush)

    def _pub(self, msg_type, topic):
        return self.create_publisher(msg_type, topic, 10) if topic else None

    def _new_bias(self, state):
        s = self.states[state]
        sig = float(s['bias_sigma'])
        return GaussMarkov(self.rng, sig, s['bias_tau'], 3) if sig > 0 else GaussMarkov(self.rng, 0, 0, 3, 0)

    def _on_truth(self, msg):
        self.truth = msg

    def _cap(self, pos):
        cap = 'fix'
        for z in self.zones:
            if math.hypot(pos[0] - z['center'][0], pos[1] - z['center'][1]) <= z['radius']:
                if QUALITY.index(z['max_state']) < QUALITY.index(cap):
                    cap = z['max_state']
        return cap

    def _set_state(self, new):
        if new == self.state:
            return
        better = QUALITY.index(new) > QUALITY.index(self.state)
        old_bias = self.bias.x.copy()
        self.state = new
        self.bias = self._new_bias(new)
        if not better:  # a degrading solution drifts away from where it was instead of jumping
            self.bias.x = old_bias
        self.get_logger().info(f'GNSS solution -> {new}')

    def _step_state(self, dt, pos):
        tr = self.transitions.get(self.state)
        if tr and self.rng.random() < 1 - math.exp(-dt / float(tr['after'])):
            names, probs = zip(*tr['to'].items())
            probs = np.array(probs, float)
            self._set_state(self.rng.choice(names, p=probs / probs.sum()))
        cap = self._cap(pos)
        if QUALITY.index(self.state) > QUALITY.index(cap):
            self._set_state(cap)

    def _epoch(self):
        if self.truth is None:
            return
        now = self.now_s()
        dt = 0.0 if self.last_epoch is None else now - self.last_epoch
        self.last_epoch = now

        p = self.truth.pose.pose.position
        q = self.truth.pose.pose.orientation
        rot = quat_to_rot([q.x, q.y, q.z, q.w])
        v_body = np.array([self.truth.twist.twist.linear.x, self.truth.twist.twist.linear.y,
                           self.truth.twist.twist.linear.z])
        w_body = np.array([self.truth.twist.twist.angular.x, self.truth.twist.twist.angular.y,
                           self.truth.twist.twist.angular.z])
        ant = np.array([p.x, p.y, p.z]) + rot @ self.antenna
        vel = rot @ (v_body + np.cross(w_body, self.antenna))

        self._step_state(dt, ant)
        s = self.states[self.state]
        bias = self.bias.step(dt)
        noise = self.rng.normal(0, [s['h_sigma'], s['h_sigma'], s['v_sigma']])
        meas = ant + bias + noise
        vel_m = vel + self.rng.normal(0, s['vel_sigma'], 3)

        h_acc = math.hypot(s['h_sigma'], s['bias_sigma']) * self.rng.uniform(0.8, 1.25)
        v_acc = math.hypot(s['v_sigma'], s['bias_sigma']) * self.rng.uniform(0.8, 1.25)
        speed = math.hypot(vel_m[0], vel_m[1])
        head_mot = math.degrees(math.atan2(vel_m[0], vel_m[1]))  # from north, clockwise
        head_acc = min(180.0, math.degrees(math.atan2(s['vel_sigma'] * 1.5, max(speed, 1e-6))))
        lat, lon, alt = world.enu_to_geodetic(self.origin, *meas)
        self.queue.append((now + self.latency, dict(
            stamp=self.truth.header.stamp, sim_t=now, state=self.state, s=s, lat=lat, lon=lon, alt=alt,
            vel=vel_m, speed=speed, head_mot=head_mot, head_acc=head_acc, h_acc=h_acc, v_acc=v_acc)))

    def _flush(self):
        now = self.now_s()
        while self.queue and self.queue[0][0] <= now:
            self._publish(self.queue.popleft()[1])

    def _publish(self, m):
        if self.pub_fix:
            fix = NavSatFix()
            fix.header.stamp, fix.header.frame_id = m['stamp'], self.frame_id
            fix.status.status = {'fix': NavSatStatus.STATUS_GBAS_FIX, 'float': NavSatStatus.STATUS_GBAS_FIX,
                                 'single': NavSatStatus.STATUS_FIX}.get(m['state'], NavSatStatus.STATUS_NO_FIX)
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude, fix.longitude, fix.altitude = m['lat'], m['lon'], m['alt']
            fix.position_covariance = [m['h_acc'] ** 2, 0.0, 0.0, 0.0, m['h_acc'] ** 2, 0.0, 0.0, 0.0, m['v_acc'] ** 2]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.pub_fix.publish(fix)
        if self.pub_vel:
            tw = TwistWithCovarianceStamped()
            tw.header.stamp, tw.header.frame_id = m['stamp'], self.frame_id
            tw.twist.twist.linear.x, tw.twist.twist.linear.y, tw.twist.twist.linear.z = (float(v) for v in m['vel'])
            var = float(m['s']['vel_sigma']) ** 2
            tw.twist.covariance = [var if i in (0, 7, 14) else 0.0 for i in range(36)]
            self.pub_vel.publish(tw)
        if self.pub_ubx:
            utc = self.utc0 + datetime.timedelta(seconds=m['sim_t'])
            gps_week_start = (utc - datetime.timedelta(days=(utc.weekday() + 1) % 7)).replace(
                hour=0, minute=0, second=0, microsecond=0)
            itow = (utc - gps_week_start).total_seconds() * 1000 + 18000  # GPS-UTC leap seconds
            fix_type = 0 if m['state'] == 'none' else 3
            frame = ubx.nav_pvt(
                itow_ms=itow, utc=utc, fix_type=fix_type, carr_soln=CARR_SOLN[m['state']],
                num_sv=m['s']['num_sv'], lat=m['lat'], lon=m['lon'], height_m=m['alt'],
                hmsl_m=m['alt'] - self.geoid, h_acc_m=m['h_acc'], v_acc_m=m['v_acc'],
                vel_ned=[m['vel'][1], m['vel'][0], -m['vel'][2]], g_speed=m['speed'],
                head_mot_deg=m['head_mot'], s_acc=m['s']['vel_sigma'], head_acc_deg=m['head_acc'])
            self.pub_ubx.publish(UInt8MultiArray(data=list(frame)))


def main():
    spin(Gnss)
