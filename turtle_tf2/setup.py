from setuptools import setup

package_name = 'turtle_tf2'

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
    maintainer_email='abilkasov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_tf2_broadcaster = turtle_tf2.turtle_tf2_broadcaster:main',
            'turtle_tf2_broadcaster_2 = turtle_tf2.turtle_tf2_broadcaster_2:main',
            'turtle_tf2_listener = turtle_tf2.turtle_tf2_listener:main',
        ],
    },
)
