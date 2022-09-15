from glob import glob
import os

from setuptools import setup

package_name = 'turtle_tf2_py'

setup(
    name=package_name,
    version='0.3.6',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shyngyskhan Abilkassov',
    author_email='abilkasov@gmail.com',
    maintainer='Alejandro Hernández Cordero, Audrow Nash',
    maintainer_email='alejandro@openrobotics.org, audrow@openrobotics.org',
    description=(
        'turtle_tf2_py demonstrates how to write a ROS2 Python tf2 broadcaster and '
        'listener with the turtlesim. The turtle_tf2_listener commands turtle2 to '
        'follow turtle1 around as you drive turtle1 using the keyboard.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_turtle_tf2_broadcaster = turtle_tf2_py.static_turtle_tf2_broadcaster:main',
            'turtle_tf2_broadcaster = turtle_tf2_py.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = turtle_tf2_py.turtle_tf2_listener:main',
            'fixed_frame_tf2_broadcaster = turtle_tf2_py.fixed_frame_tf2_broadcaster:main',
            'dynamic_frame_tf2_broadcaster = turtle_tf2_py.dynamic_frame_tf2_broadcaster:main',
            'turtle_tf2_message_broadcaster = turtle_tf2_py.turtle_tf2_message_broadcaster:main',
        ],
    },
)
