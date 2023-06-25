
# python launch modules
from launch import LaunchDescription
from launch_ros.actions import Node


# 
# Launch Description

def generate_launch_description():


    # definition of the node 
    turtlesim_node_1 = Node(
        package='turtlesim',
        namespace='turtlesim1',
        executable='turtlesim_node',
        name='sim'
    )

    # changing the namespace it is possible to start both
    turtlesim_node_2 = Node(
        package='turtlesim',
        namespace='turtlesim2',
        executable='turtlesim_node',
        name='sim'
    )

    mimic_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )

    #--ros-args --remap turtle1/cmd_vel:=turtlesim1/turtle1/cmd_vel
    return LaunchDescription([
        turtlesim_node_1,
        turtlesim_node_2,
        mimic_node
    ])