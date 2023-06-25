from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    talker = Node(
            package='demo_nodes_cpp',
            executable='talker'
            )

    listener = Node(
            package='demo_nodes_cpp',
            executable='listener',
    )
    
    return LaunchDescription([
        talker,
        listener
    ])
