from glob import glob

from setuptools import find_packages, setup

package_name = 'sim_env'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kristijan',
    maintainer_email='kikomali2517@gmail.com',
    description='Robot-agnostic Gazebo simulation environment with device models and micro-ROS HIL.',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [
            'build_scene = sim_env.scene:main',
            'gnss_model = sim_env.devices.gnss:main',
            'imu_model = sim_env.devices.imu:main',
            'stepper_model = sim_env.devices.stepper:main',
            'diff_drive_mixer = sim_env.devices.diff_drive:main',
        ],
    },
)
