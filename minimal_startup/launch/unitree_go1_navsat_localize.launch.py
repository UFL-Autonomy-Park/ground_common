import launch_ros.actions
import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    file_path = os.path.expanduser("~/robot_param.yaml")
    with open(file_path, "r") as stream:
        try:
            data_loaded = yaml.safe_load(stream)
            robot_namespace = '/'+ data_loaded['robot_namespace']
        except yaml.YAMLError as exc:
            print(exc)

    setup_path = LaunchConfiguration('setup_path')

    arg_setup_path = DeclareLaunchArgument('setup_path', default_value='/home/administrator/platform_ws/src/common/minimal_startup/')

    arg_output_final_position = DeclareLaunchArgument(
        'output_final_position',
        default_value='false')

    arg_output_file = DeclareLaunchArgument(
        'output_location',
    default_value='~/dual_ekf_navsat_debug.txt')

    dir_params = PathJoinSubstitution([
        setup_path, 'params'])

    config_localization = [
        dir_params,
        '/dual_ekf_navsat_test.yaml'
    ]

    return LaunchDescription([
    arg_output_file,
    arg_output_final_position,
    arg_setup_path,
    launch_ros.actions.Node(
            package='robot_localization',
            namespace=robot_namespace,
            executable='ekf_node',
            name='ekf_local',
        output='screen',
            parameters=[{"frequency": 30.0},
                        {"sensor_timeout": 1.0},
                        {"two_d_mode": True},
                        #{"transform_time_offset": 0.0},
                        #{"transform_timeout": 0.0},
                        {"print_diagnostics": True},
                        #{"debug": False},
                        {"two_d_mode": True},
                        {"map_frame": 'autonomy_park'},
                        {"odom_frame": 'odom'},
                        {"base_link_frame": 'base_link'},
                        {"world_frame": 'odom'},
                        {"publish_tf" : True},
                        {"imu0": '/go1_0165/sensors/imu_0/data'},
                        {"imu0_config": [False, False, False,
                                         True,  True,  True,
                                         False, False, False,
                                         True,  True,  True,
                                         True,  True,  True]},
                        {"imu0_differential": False},
                        {"imu0_relative": False},
                        {"imu0_queue_size": 10},
                        {"imu0_remove_gravitational_acceleration": True},
                        {"twist0": "/go1_0165/nonholo_cmd_vel_stamped"},  # Replace with your twist command topic (e.g., velocity commands)
                        {"twist0_config": [False, False, False,  # x, y, z position (not used in twist messages)
                                           False, False, False,    # r, p, y, orientation
                                           True, True, True,    # x, y, z lin velocities
                                           False, False, False,    #ang vel
                                           False, False, False]}, #lin acc
                        {"twist0_differential": False},  # True if your twist message is in differential form (e.g., wheel speeds)
                        {"twist0_queue_size": 10},  # Size of the Twist message queue
                        {"use_control": False},
                        {"use_odometry": False},
                        {"use_navsat": False},
                       ],
            remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/diagnostics', 'diagnostics'),
                        #('imu', 'sensors/imu_0/data'),
                        #('odom', ''),
                        ('odometry/filtered', 'odometry/local'),
                        ]
           ),
    launch_ros.actions.Node(
            package='robot_localization',
            namespace=robot_namespace,
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[#config_localization,
                        {"frequency": 30.0},
                        {"sensor_timeout": 0.1},
                        {"two_d_mode": True},
                        #{"transform_time_offset": 0.0},
                        #{"transform_timeout": 0.0},
                        {"print_diagnostics": True},
                        # {"debug": False},
                        {"map_frame": 'autonomy_park'},
                        {"odom_frame": 'odom'},
                        {"base_link_frame": 'base_link'},
                        {"world_frame": 'autonomy_park'},
                        {"publish_tf" : True},
                        {"twist0": "/go1_0165/nonholo_cmd_vel_stamped"},  # Replace with your twist command topic (e.g., velocity commands)
                        {"twist0_config": [False, False, False,  # x, y, z position (not used in twist messages)
                                           False, False, False,    # r, p, y orientation
                                           True, True, True,    # x, y, z linear velocities
                                           False, False, False,    # rd, pd, yd angular vel
                                           False, False, False]},  # lin acc
                        {"twist0_differential": False},  # True if your twist message is in differential form (e.g., wheel speeds)
                        {"twist0_queue_size": 10},  # Size of the Twist message queue
                        {"odom0": "odometry/gps"},
                        {"odom0_config": [True, True, False,
                                          False, False, False,
                                          False, False, False,
                                          False, False, False,
                                          False, False, False]},
                        {"odom0_differential": False},
                        {"odom0_relative": False},
                        {"odom0_queue_size": 10},
                        {"imu0": "sensors/imu_0/data"},
                        {"imu0_config": [False, False, False,
                                         True,  True,  True,
                                         False, False, False,
                                         True,  True,  True,
                                         True,  True,  True]},
                        {"imu0_differential": False},
                        {"imu0_relative": False},
                        {"imu0_queue_size": 10},
                        {"imu0_remove_gravitational_acceleration": True},
                        {"use_control": False},
                        #{"process_noise_covariance": [1e-3, 1e-3, 1e-3, 0.3, 0.3, 0.01, 0.5, 0.5, 0.1, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]},
                        #{"initial_estimate_covariance": [1e-9, 1e-9, 1e-9, 1.0, 1.0, 1e-9, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]},
                        ],
            remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/diagnostics', 'diagnostics'),
                        #('imu', 'sensors/imu_0/data'),
                        #('odom', 'platform/odom'),
                        ('odometry/filtered', 'odometry/global'),
                        ]
           ),
    launch_ros.actions.Node(
            package='robot_localization',
            namespace=robot_namespace,
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[#config_localization,
                        {"frequency": 30.0},
                        {"delay": 3.0},
                        {"map_frame": "autonomy_park"},
                        {"odom_frame": "odom"},
                        {"world_frame": "autonomy_park"},
                        {"base_link_frame": "base_link"},
                        {"magnetic_declination_radians": 0.1117},  # For lat/long 29.628164, -82.360346
                        {"yaw_offset": 0.0},  # 1.570796327 IMU reads 0 facing magnetic north, not east
                        {"zero_altitude": False},
                        {"broadcast_cartesian_transform": True},
                        {"publish_filtered_gps": True},
                        {"use_odometry_yaw": False},
                        {"wait_for_datum": True},
                        {"datum": [29.628164, -82.360346, -0.602196554855]}],
            remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                        ('/diagnostics', 'diagnostics'),
                        ('imu', 'sensors/imu_0/data'),
                        ('gps/fix', 'sensors/gps/fix'),
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global'),
                        ],
            ),
])
