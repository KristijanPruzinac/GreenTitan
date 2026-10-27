from glob import glob

from setuptools import find_packages, setup

package_name = 'greentitan_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kristijan',
    maintainer_email='kikomali2517@gmail.com',
    description='GreenTitan mower simulation and hardware-in-the-loop on sim_env.',
    license='GPL-3.0-only',
    entry_points={'console_scripts': ['console = greentitan_sim.console:main']},
)
