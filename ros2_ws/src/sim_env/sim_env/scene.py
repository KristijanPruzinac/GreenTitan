"""Build a complete scene (world + robot + bridge config) from YAML into the cache directory.

    ros2 run sim_env build_scene --world world.yaml --robot robot.yaml [--out DIR]

Prints the generated paths as YAML. The launch file calls ``build()`` directly.
"""
import argparse
import hashlib
import json
import os

import yaml

from . import config, robot, world


def _bridge_entries(iface, world_name):
    t = iface['topics']
    entries = [
        {'ros_topic_name': '/clock', 'gz_topic_name': f'/world/{world_name}/clock',
         'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': t['ground_truth'], 'gz_topic_name': t['ground_truth'],
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': t['joint_states'], 'gz_topic_name': t['joint_states'],
         'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model', 'direction': 'GZ_TO_ROS'},
    ]
    for j in iface['joints']:
        entries.append({'ros_topic_name': t['joint_cmd'](j), 'gz_topic_name': t['joint_cmd'](j),
                        'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
                        'direction': 'ROS_TO_GZ'})
    types = {'imu': ('sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
             'camera': ('sensor_msgs/msg/Image', 'gz.msgs.Image'),
             'gpu_lidar': ('sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')}
    for s in iface['sensors']:
        ros_type, gz_type = types[s['type']]
        topic = t['sensor'](s['name'])
        entries.append({'ros_topic_name': topic, 'gz_topic_name': topic, 'ros_type_name': ros_type,
                        'gz_type_name': gz_type, 'direction': 'GZ_TO_ROS', 'lazy': s['type'] == 'camera'})
    return entries


def build(world_file, robot_file, out_dir=None):
    world_cfg = config.load(world_file)
    robot_cfg = config.load(robot_file)
    if out_dir is None:
        digest = hashlib.sha1(json.dumps([world_cfg, robot_cfg], sort_keys=True, default=str).encode()).hexdigest()
        out_dir = config.cache_dir('scenes', digest[:12])
    os.makedirs(out_dir, exist_ok=True)

    if any(s['type'] in ('camera', 'gpu_lidar') for s in robot_cfg['robot'].get('sensors', [])):
        world_cfg.setdefault('world', {})['rendering'] = True
    scene = world.build(world_cfg, out_dir)
    iface = robot.build(robot_cfg, out_dir)

    spawn = iface['spawn']
    x, y = spawn.get('position', [0.0, 0.0])
    yaw = float(spawn.get('yaw', 0.0))
    z = world.ground_z(scene, x, y) - iface['lowest_point'] + float(spawn.get('clearance', 0.02))

    bridge_path = os.path.join(out_dir, 'bridge.yaml')
    with open(bridge_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(_bridge_entries(iface, scene['world_name']), f, sort_keys=False)

    return {
        'out_dir': out_dir,
        'world_name': scene['world_name'],
        'world_sdf': scene['world_sdf'],
        'robot_name': iface['name'],
        'robot_sdf': iface['sdf'],
        'bridge_yaml': bridge_path,
        'spawn': [x, y, z, 0.0, 0.0, yaw],
        'origin': scene['origin'],
        'joints': iface['joints'],
        'ground_truth_topic': iface['topics']['ground_truth'],
        'joint_cmd_topics': {j: iface['topics']['joint_cmd'](j) for j in iface['joints']},
        'sensor_topics': {s['name']: iface['topics']['sensor'](s['name']) for s in iface['sensors']},
        'world_cfg': world_cfg,
        'robot_cfg': robot_cfg,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', required=True)
    ap.add_argument('--robot', required=True)
    ap.add_argument('--out')
    a = ap.parse_args()
    s = build(a.world, a.robot, a.out)
    print(yaml.safe_dump({k: v for k, v in s.items() if not k.endswith('_cfg')}, sort_keys=False))


if __name__ == '__main__':
    main()
