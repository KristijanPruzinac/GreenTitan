"""Generic simulation launch: world + robot from YAML, Gazebo, ROS bridge and device models.

    ros2 launch sim_env sim.launch.py world:=<world.yaml> robot:=<robot.yaml> [gui:=true]

Device models are listed in the robot file under ``devices:``; each entry is
``{model: gnss|imu|stepper|diff_drive, name: ..., <config>}`` or
``{package: pkg, executable: exe, name: ..., <config>}`` for devices from other packages.
Each node receives its entry (plus scene facts under ``scene``) as the JSON parameter ``config``.
"""
import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim_env import scene as scene_builder

BUILTIN_DEVICES = {'gnss': 'gnss_model', 'imu': 'imu_model', 'stepper': 'stepper_model',
                   'diff_drive': 'diff_drive_mixer'}


def _scene_facts(s):
    return {
        'origin': s['origin'],
        'world_name': s['world_name'],
        'robot_name': s['robot_name'],
        'ground_truth_topic': s['ground_truth_topic'],
        'joint_cmd_topics': s['joint_cmd_topics'],
        'sensor_topics': s['sensor_topics'],
        'wheels': {w['name']: w for w in s['robot_cfg']['robot'].get('wheels', [])},
        'gnss_zones': s['world_cfg'].get('gnss_zones', []),
    }


def device_nodes(s, use_sim_time=True):
    facts = _scene_facts(s)
    nodes = []
    for dev in s['robot_cfg'].get('devices', []):
        dev = dict(dev)
        package = dev.pop('package', 'sim_env')
        executable = dev.pop('executable', None) or BUILTIN_DEVICES[dev.pop('model')]
        name = dev.pop('name', executable)
        dev.pop('model', None)
        dev['scene'] = facts
        nodes.append(Node(package=package, executable=executable, name=name, output='screen',
                          parameters=[{'config': json.dumps(dev), 'use_sim_time': use_sim_time}]))
    return nodes


def _setup(context):
    world_file = LaunchConfiguration('world').perform(context)
    robot_file = LaunchConfiguration('robot').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    sim_time = LaunchConfiguration('sim_time').perform(context).lower() == 'true'
    s = scene_builder.build(world_file, robot_file)
    x, y, z, _, _, yaw = s['spawn']

    gz_cmd = ['gz', 'sim', '-r', s['world_sdf']] if gui else ['gz', 'sim', '-r', '-s', s['world_sdf']]
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               s['out_dir'] + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', s['world_name'], '-file', s['robot_sdf'], '-name', s['robot_name'],
                        '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge', output='screen',
             parameters=[{'config_file': s['bridge_yaml'], 'use_sim_time': sim_time}]),
    ]
    return actions + device_nodes(s, sim_time)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', description='world YAML'),
        DeclareLaunchArgument('robot', description='robot YAML'),
        DeclareLaunchArgument('gui', default_value='false', description='start the Gazebo GUI'),
        # false: nodes run on wall time and /clock is not bridged (it runs at the physics rate,
        # ~1 kHz, and costs each Python node most of a core). Only valid while the sim runs at RTF 1.
        DeclareLaunchArgument('sim_time', default_value='true', description='use /clock in the sim nodes'),
        # gz-transport defaults to the WSL/LAN interface; loopback keeps discovery reliable and local.
        SetEnvironmentVariable('GZ_IP', os.environ.get('GZ_IP', '127.0.0.1')),
        OpaqueFunction(function=_setup),
    ])
