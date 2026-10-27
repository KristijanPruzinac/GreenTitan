"""Firmware debug console over ROS: prints esp/log, sends typed keys to esp/console.

    ros2 run greentitan_sim console              # interactive: each key is sent immediately
    ros2 run greentitan_sim console -- m w w g   # send these keys (0.3 s apart) and keep printing
"""
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('esp_console')
    pub = node.create_publisher(String, 'esp/console', 10)
    node.create_subscription(String, 'esp/log', lambda m: (sys.stdout.write(m.data.replace('\r', '')),
                                                           sys.stdout.flush()), 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    keys = [a for a in sys.argv[1:] if a != '--']
    try:
        if keys:
            for _ in range(100):  # wait for the firmware subscriber, else the first keys are lost
                if pub.get_subscription_count() > 0:
                    break
                time.sleep(0.05)
            time.sleep(0.3)
            for k in keys:
                pub.publish(String(data=k.replace('space', ' ')))
                time.sleep(0.3)
            while rclpy.ok():
                time.sleep(0.2)
        else:
            old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            try:
                while rclpy.ok():
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        c = sys.stdin.read(1)
                        if c == '\x03':
                            break
                        pub.publish(String(data=c))
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
