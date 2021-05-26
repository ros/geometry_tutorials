from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtle_tf2',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]

        ),
	Node(
	    package='turtle_tf2',
	    executable='turtle_tf2_broadcaster',
	    name='broadcaster2',
	    parameters=[
		{'turtlename': 'turtle2'}
	    ]

        ),
        Node(
            package='turtle_tf2',
            executable='turtle_tf2_listener',
            name='listener'
        ),
    ])
