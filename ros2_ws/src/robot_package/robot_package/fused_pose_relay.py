"""EKF pose for the ESP32 in a compact message.

The EKF's Odometry is 724 bytes, 576 of them covariance. Over the micro-ROS serial link (MTU 512)
the agent sometimes sent it on a stream that cannot split it, failed ("serialization error") and
dropped the session: no pose on the ESP for 15-50 s (HIL 2026-10-28). The ESP only needs these
values, so it gets them as one Float64MultiArray of FIELDS, best effort (an old pose is no use),
one per EKF output (20-30 Hz with the EKF at 30 Hz, 2026-11-02):

    [stamp_sec, stamp_nanosec, x, y, yaw, yaw_variance, vx, vy, omega]

vx, vy are in the body frame like the Odometry twist.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

FIELDS = 9


class FusedPoseRelay(Node):
    def __init__(self):
        super().__init__('fused_pose_relay')
        self.pub = self.create_publisher(Float64MultiArray, 'esp/fused_pose', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered/global', self.on_odom, 10)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        out = Float64MultiArray()
        out.data = [float(m.header.stamp.sec), float(m.header.stamp.nanosec),
                    m.pose.pose.position.x, m.pose.pose.position.y, yaw, m.pose.covariance[35],
                    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z]
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(FusedPoseRelay())


if __name__ == '__main__':
    main()
