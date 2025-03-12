import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   sensors_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('minimal_startup')),
         '/sensors_service.launch.py'])
      )
   holonomic_interpreter = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('holonomic_interpreter'), 'launch'),
         '/holonomic_interpreter.launch.py'])
      )

   robot_localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('minimal_startup')),
         '/navsat_localize.launch.py'])
     )

   static_transform_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('minimal_startup')),
         '/clearpath_static_transform.launch.py'])
     )
   avoidance_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('avoidance'), 'launch'),
         '/avoidance.launch.py'])
     )


   return LaunchDescription([
      #holonomic_interpreter,
      avoidance_launch,
      sensors_launch,
      robot_localization_launch,
      static_transform_launch,
   ])
