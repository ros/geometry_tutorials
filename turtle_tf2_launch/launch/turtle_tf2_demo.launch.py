from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	Node(
            package='turtlesim',
            # namespace='turtle1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_tf2',
            # namespace='turtle1',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1'
        ),
        Node(
            package='turtle_tf2',
            # namespace='turtle2',
            executable='turtle_tf2_broadcaster_2',
            name='broadcaster2'
        ),
        Node(
            package='turtle_tf2',
            # namespace='turtle2',
            executable='turtle_tf2_listener',
            name='listener'
        ),
    ])
