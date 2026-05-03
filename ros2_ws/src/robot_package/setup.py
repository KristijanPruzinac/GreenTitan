from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/robot_package']),
        ('share/robot_package', ['package.xml']),
        ('share/robot_package/launch', glob('launch/*')),
        ('share/robot_package/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kristijan',
    maintainer_email='kikomali2517@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'datum_relay = robot_package.datum_relay:main',
            'path_visualizer = robot_package.path_visualizer:main',
        ],
    },
)
