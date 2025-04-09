from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='duckiebot',
            namespace='duckietown',
            executable='duckiebot_node',
            name='duckiebot_node1'
        ),
        Node(
            package='control',
            namespace='duckietown',
            executable='control_node',
            name='control_node1'
        )
    ])
