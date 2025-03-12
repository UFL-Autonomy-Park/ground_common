import os
import launch_ros
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

   file_path = os.path.expanduser("~/robot_param.yaml")
   with open(file_path, "r") as stream:
     try:
         data_loaded = yaml.safe_load(stream)
         robot_namespace = '/'+ data_loaded['robot_namespace']
         robot_name = data_loaded['robot_name']

     except yaml.YAMLError as exc:
         print(exc)

   sensors_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('minimal_startup')),
         '/unitree_sensors_service.launch.py'])
      )

   holonomic_interpreter = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('holonomic_interpreter'), 'launch'),
         '/holonomic_interpreter.launch.py'])
      )

   robot_localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('minimal_startup')),
         '/unitree_go1_navsat_localize.launch.py'])
     )

   static_transform_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('minimal_startup')),
         '/go1_static_transform.launch.py'])
     )

   return LaunchDescription([
      holonomic_interpreter,
      sensors_launch,
      robot_localization_launch,
      static_transform_launch,
      launch_ros.actions.Node(
        package='unitree_velocity_interface',
        namespace=robot_namespace,
        executable='unitree_velocity_interface_node',
        name='unitree_velocity_interface_node',
        output='screen',
        parameters=[
        ],
        remappings=[
        ]
      ),
      launch_ros.actions.Node(
        package='unitree_legged_real',
        namespace=robot_namespace,
        executable='udp',
        name='udp',
        output='screen',
        parameters=[
        ],
        remappings=[
        ]
      ),
   ])