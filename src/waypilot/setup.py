from glob import glob

from setuptools import find_packages, setup

package_name = 'waypilot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rakshitha H S',
    maintainer_email='201125347+Rakshithahs1216@users.noreply.github.com',
    description='Waypoint follower for TurtleBot with a web UI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = waypilot.navigator_node:main',
            'mission_manager = waypilot.mission_manager_node:main',
            'web_bridge = waypilot.web_bridge_node:main',
        ],
    },
)
