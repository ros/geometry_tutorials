from setuptools import setup

package_name = 'dummy_robot_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shyngys',
    maintainer_email='shyngys@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = dummy_robot_tf.turtle_tf2_broadcaster:main',
            'turtle_tf2_broadcaster_2 = dummy_robot_tf.turtle_tf2_broadcaster2:main',
            'turtle_tf2_listener = dummy_robot_tf.turtle_tf2_listener:main',
        ],
    },
)
