import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum

class DatumRelay(Node):
    def __init__(self):
        super().__init__('datum_relay')
        self.datum_set = False
        self.client = self.create_client(SetDatum, '/datum')
        self.sub = self.create_subscription(NavSatFix, '/esp/datum', self.on_datum, 10)
        self.timer = self.create_timer(5.0, self.on_timer)
        self.get_logger().info('Datum relay waiting for ESP32...')

    def on_timer(self):
        if not self.datum_set:
            self.get_logger().info('Still waiting for datum from ESP32...')

    def on_datum(self, msg):
        if self.datum_set:
            return

        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetDatum service not available')
            return

        # HACK: ESP32 firmware smuggles yaw (radians, ENU) through NavSatFix.altitude
        # because geographic_msgs/GeoPoseStamped isn't in the default micro-ROS message set.
        yaw = msg.altitude

        req = SetDatum.Request()
        req.geo_pose.position.latitude  = msg.latitude
        req.geo_pose.position.longitude = msg.longitude
        req.geo_pose.position.altitude  = 0.0
        # Yaw → quaternion about Z (ENU): (0, 0, sin(yaw/2), cos(yaw/2))
        req.geo_pose.orientation.x = 0.0
        req.geo_pose.orientation.y = 0.0
        req.geo_pose.orientation.z = math.sin(yaw / 2.0)
        req.geo_pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Setting datum: lat={msg.latitude:.7f} lon={msg.longitude:.7f} yaw={yaw:.4f} rad'
        )

        future = self.client.call_async(req)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        self.datum_set = True
        self.timer.cancel()
        self.get_logger().info('Datum set successfully')

def main():
    rclpy.init()
    try:
        rclpy.spin(DatumRelay())
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
