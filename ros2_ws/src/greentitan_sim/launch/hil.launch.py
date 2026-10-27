"""GreenTitan hardware-in-the-loop: lawn + mower in Gazebo, the real ESP32 firmware (env
esp32dev_hil) over micro-ROS, and the robot_package EKF.

    ros2 launch greentitan_sim hil.launch.py device:=/dev/ttyUSB0          # native Linux
    ros2 launch greentitan_sim hil.launch.py tcp_port:=8888                 # WSL2 + serial_tcp_bridge.py on Windows

Without firmware (drive with teleop_twist_keyboard):
    ros2 launch greentitan_sim hil.launch.py robot:=<share>/config/mower_teleop.yaml agent:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sim_env'), 'launch', 'sim.launch.py')),
        launch_arguments={'world': arg('world'), 'robot': arg('robot'), 'gui': arg('gui')}.items())]

    if arg('agent') == 'true':
        device = arg('device')
        if not device:  # serial arrives over TCP from the Windows relay; expose it as a pty
            device = os.path.join(os.environ.get('XDG_RUNTIME_DIR', '/tmp'), 'greentitan_esp32')
            actions.append(ExecuteProcess(
                cmd=['socat', f'pty,link={device},raw,echo=0', f"tcp-listen:{arg('tcp_port')},reuseaddr"],
                output='screen', respawn=True, respawn_delay=1.0))
        actions.append(Node(package='micro_ros_agent', executable='micro_ros_agent', output='screen',
                            arguments=['serial', '--dev', device, '-b', arg('baud')],
                            respawn=True, respawn_delay=2.0))

    if arg('localization') == 'true':
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_package'), 'launch', 'robot.launch.xml'))))
    if arg('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', output='log'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('greentitan_sim')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'config', 'lawn.yaml')),
        DeclareLaunchArgument('robot', default_value=os.path.join(share, 'config', 'mower.yaml')),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('agent', default_value='true', description='start the micro-ROS agent'),
        DeclareLaunchArgument('device', default_value='', description='ESP32 serial device; empty = use tcp_port'),
        DeclareLaunchArgument('tcp_port', default_value='8888'),
        DeclareLaunchArgument('baud', default_value='921600'),
        DeclareLaunchArgument('localization', default_value='true', description='start the robot_package EKF'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
