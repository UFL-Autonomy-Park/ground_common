import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
         Node(
            package='avoidance',
            executable='avoidance_node',
            name='avoidance_node',
            namespace='a200_0706',
            parameters=[os.path.join(get_package_share_directory('px4_safety_lib'), 'param', 'safety_config.yaml')],
	   #remappings=[('cmd_vel_filtered', 'cmd_vel')],
	    output='screen'
        )
    ])
