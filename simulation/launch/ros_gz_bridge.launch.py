# Launches the ros_gz_bridge with some default values for convenience

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('simulation'),
                               'config/ros_gz_bridge.yaml')

    # args that can be set from the command line or a default will be used
    bridge_name_launch_arg = DeclareLaunchArgument(
        "bridge_name", default_value=TextSubstitution(text="ros_gz_bridge")
    )
    config_file_launch_arg = DeclareLaunchArgument(
        "config_file", default_value=TextSubstitution(text=config_file)
    )
    log_level_launch_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="info")
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_bridge'), 'launch'),
                '/ros_gz_bridge.launch.py']),
        launch_arguments={'bridge_name': LaunchConfiguration('bridge_name'),
                          'config_file': LaunchConfiguration('config_file'),
                          'log_level': LaunchConfiguration('log_level')}.items(),
        )

    return LaunchDescription([
        bridge_name_launch_arg,
        config_file_launch_arg,
        log_level_launch_arg,
        ros_gz_bridge
    ])
