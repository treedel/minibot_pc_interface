from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'libbot_navigation'
    package_share = FindPackageShare(package=package_name).find(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_config_path = LaunchConfiguration('nav2_config_path')
    map_path = LaunchConfiguration('map_path')
    twist_mux_config_path = LaunchConfiguration('twist_mux_config_path')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_path = DeclareLaunchArgument(
        name='map_path',
        default_value=os.path.join(package_share, 'maps', 'vnest_sav.yaml'),
        description='Location of map file for AMCL localization and navigation'
    )

    declare_nav2_config_path = DeclareLaunchArgument(
        name='nav2_config_path',
        default_value=os.path.join(package_share, 'config', 'nav2_params.yaml'),
        description='Location of "nav2_params.yaml" file for nav2 parameters'
    )

    declare_twist_mux_config_path = DeclareLaunchArgument(
        name='twist_mux_config_path',
        default_value=os.path.join(package_share, 'config', 'twist_mux.yaml'),
        description='Location of "twist_mux.yaml" file for merging nav2 and manual command override'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'rviz_config.rviz'),
        description='Location of RViz config file'
    )

    package_nav2_bringup = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': nav2_config_path,
            'use_sim_time': use_sim_time
        }.items()
    )

    package_nav2_bringup = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_config_path,
            'map_subscribe_transient_local': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        arguments=['--ros-args', '--params-file', [twist_mux_config_path]],
        output='screen'
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_path)
    ld.add_action(declare_nav2_config_path)
    ld.add_action(declare_twist_mux_config_path)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)

    ld.add_action(amcl)
    ld.add_action(nav2)
    ld.add_action(twist_mux)
    ld.add_action(rviz)

    return ld