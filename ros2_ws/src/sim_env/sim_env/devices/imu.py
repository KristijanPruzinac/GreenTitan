"""MEMS IMU model.

Takes the simulator's ideal IMU (specific force incl. gravity, body rates, in the sensor frame as
mounted) and adds what a cheap MEMS part does: turn-on bias, bias instability (Gauss-Markov),
white noise from a noise density, scale-factor error, full-scale saturation, ADC quantisation
and an optional low-pass (digital LPF), plus vibration from motors as band-limited noise.

Config::

    input: imu                         # simulator sensor name (topic injected by the launch file)
    output_topic: imu/raw
    frame_id: imu
    orientation: false                 # publish the simulator's orientation (true) or mark unknown
    gyro_angle: false                  # instead put the output gyro z integrated at the input rate
                                       # in orientation (a z rotation): a HIL link carries samples in
                                       # bursts, the receiver turns it back into the mean rate
    lpf_hz: 44                         # 0 disables
    output_rate: 0                     # Hz, decimate after the LPF (0 = every input sample)
    gyro:  {noise_density: 8.7e-5, bias_sigma: 0.03, bias_instability: 0.0005, bias_tau: 100,
            scale_error: 0.01, range: 8.73, resolution: 0.000133}            # SI units (rad/s)
    accel: {noise_density: 0.004, bias_sigma: 0.05, bias_instability: 0.002, bias_tau: 100,
            scale_error: 0.01, range: 19.6, resolution: 0.000598}             # m/s^2
    vibration: {accel_sigma: 0.0, gyro_sigma: 0.0}
"""
import time

import numpy as np
from sensor_msgs.msg import Imu

from .base import SENSOR_QOS, DeviceNode, GaussMarkov, spin

DEFAULTS = {  # roughly an MPU6050 at +-500 dps / +-2 g
    'gyro': {'noise_density': 8.7e-5, 'bias_sigma': 0.03, 'bias_instability': 0.0005,
             'bias_tau': 100.0, 'scale_error': 0.01, 'range': 8.73, 'resolution': 0.000133},
    'accel': {'noise_density': 0.004, 'bias_sigma': 0.05, 'bias_instability': 0.002, 'bias_tau': 100.0,
              'scale_error': 0.01, 'range': 19.6, 'resolution': 0.000598},
}


class Channel:
    def __init__(self, rng, p):
        self.p = p
        self.rng = rng
        self.turn_on = rng.normal(0, float(p['bias_sigma']), 3)
        self.drift = GaussMarkov(rng, float(p['bias_instability']), float(p['bias_tau']), 3)
        self.scale = 1.0 + rng.normal(0, float(p['scale_error']), 3)
        self.lp = None

    def apply(self, ideal, dt, rate, vib_sigma, lpf_alpha):
        white = self.rng.normal(0, float(self.p['noise_density']) * np.sqrt(max(rate, 1.0) / 2), 3)
        vib = self.rng.normal(0, vib_sigma, 3) if vib_sigma > 0 else 0.0
        self.bias = self.turn_on + self.drift.step(dt)
        y = ideal * self.scale + self.bias + white + vib
        if lpf_alpha is not None:
            self.lp = y if self.lp is None else self.lp + lpf_alpha * (y - self.lp)
            y = self.lp
        rng = float(self.p['range'])
        y = np.clip(y, -rng, rng)
        res = float(self.p.get('resolution', 0))
        return np.round(y / res) * res if res > 0 else y


class ImuModel(DeviceNode):
    def __init__(self):
        super().__init__('imu_model')
        merge = lambda k: {**DEFAULTS[k], **(self.c(k) or {})}  # noqa: E731
        self.gyro = Channel(self.rng, merge('gyro'))
        self.accel = Channel(self.rng, merge('accel'))
        self.vib = self.c('vibration', {})
        self.lpf_hz = float(self.c('lpf_hz', 0))
        self.frame_id = self.c('frame_id', 'imu')
        self.orientation = bool(self.c('orientation', False))
        self.gyro_angle = bool(self.c('gyro_angle', False))
        self.angle = 0.0
        self.prev_wz = None
        self.prev_wall = 0.0
        self.last = None
        self.last_pub = None
        self.output_period = 1.0 / float(self.c('output_rate', 0)) if self.c('output_rate') else 0.0
        self.pub = self.create_publisher(Imu, self.c('output_topic', 'imu/raw'), SENSOR_QOS)
        self.create_subscription(Imu, self.scene['sensor_topics'][self.c('input', 'imu')], self._on_imu, SENSOR_QOS)

    def _on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last is None else max(t - self.last, 0.0)
        self.last = t
        rate = 1.0 / dt if dt > 0 else 100.0
        alpha = None
        if self.lpf_hz > 0 and dt > 0:
            alpha = 1 - np.exp(-2 * np.pi * self.lpf_hz * dt)
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w = self.gyro.apply(w, dt, rate, float(self.vib.get('gyro_sigma', 0)), alpha)
        a = self.accel.apply(a, dt, rate, float(self.vib.get('accel_sigma', 0)), alpha)
        # The receiver scales the angle rate by the sim speed (it integrates over wall time). That is
        # right for the rotation but not for the bias, which a real part shows per wall second: the
        # bias is integrated over wall time here, so off real time it stays constant at the receiver
        # (2026-11-02: at 0.87x during startup the boot calibration came out 0.26 deg/s off)
        wall = time.monotonic()
        bz = float(self.gyro.bias[2])
        if self.prev_wz is not None:
            self.angle += 0.5 * (self.prev_wz + float(w[2]) - bz) * dt + bz * min(wall - self.prev_wall, 1.0)
        self.prev_wz, self.prev_wall = float(w[2]) - bz, wall
        if self.last_pub is not None and t - self.last_pub < self.output_period - 1e-6:
            return
        self.last_pub = t

        out = Imu()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame_id
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = (float(v) for v in w)
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (float(v) for v in a)
        gv = (float(self.gyro.p['noise_density']) ** 2) * rate / 2
        av = (float(self.accel.p['noise_density']) ** 2) * rate / 2
        out.angular_velocity_covariance = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        out.linear_acceleration_covariance = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        if self.gyro_angle:
            out.orientation.z, out.orientation.w = float(np.sin(self.angle / 2)), float(np.cos(self.angle / 2))
        elif self.orientation:
            out.orientation = msg.orientation
        else:
            out.orientation_covariance[0] = -1.0
        self.pub.publish(out)


def main():
    spin(ImuModel)
