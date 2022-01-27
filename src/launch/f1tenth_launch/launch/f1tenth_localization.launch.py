from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os

def generate_launch_description():
    nav2_pkg = get_package_share_directory('nav2_bringup')
    f1tenth_launch_pkg = get_package_share_directory('f1tenth_launch')

    localization_param_file = os.path.join(f1tenth_launch_pkg, 'param/amcl.param.yaml')
    localization_param = DeclareLaunchArgument(
        'localization_param_file',
        default_value=localization_param_file,
        description='Path to config file for localization nodes'
    )

    map_file_path = os.path.join(f1tenth_launch_pkg, 'data/red_bull_ring_racetrack.yaml')
    map_file = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Path to 2D map config file'
    )

    print(LaunchConfiguration("map_file"))
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg,
                         'launch/localization_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('localization_param_file'),
            'map': LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        # localization_param_file,
        localization_param,
        map_file,
        nav2_localization,
    ])