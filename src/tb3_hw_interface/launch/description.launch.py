from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import os
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'tb3_hw_interface'
    urdf_filename = 'tb3_hw.urdf.xacro'
    package_share = FindPackageShare(package=package_name).find(package_name)

    urdf_path = os.path.join(package_share, 'urdf', urdf_filename)

    robot_description = xacro.process_file(urdf_path).toxml()

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = LaunchConfiguration('rviz_config_path')
    use_rsp = LaunchConfiguration('use_rsp')

    declare_use_rsp = DeclareLaunchArgument(
        name='use_rsp',
        default_value='true',
        description='Whether to launch robot state publisher'
    )

    declare_use_rviz = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name='rviz_config_path',
        default_value=os.path.join(package_share, 'config', 'model.rviz'),
        description='Location of RViz config file'
    )

    robot_state_publisher = Node(
        condition=IfCondition(use_rsp),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
        }],
        output='screen'
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_rsp)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_path)

    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)

    return ld