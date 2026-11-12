# dom_scan.py: which ROS domains carry an esp/* topic (finds where a micro-ROS board landed)
import time, rclpy
from rclpy.context import Context
nodes = []
for d in range(0, 61):
    c = Context(); rclpy.init(context=c, domain_id=d)
    nodes.append((d, rclpy.create_node(f'scan{d}', context=c)))
time.sleep(5)
for d, n in nodes:
    t = [name for name, _ in n.get_topic_names_and_types() if 'esp' in name or 'imu/data' in name]
    if t: print(d, t)
print('scan done')
