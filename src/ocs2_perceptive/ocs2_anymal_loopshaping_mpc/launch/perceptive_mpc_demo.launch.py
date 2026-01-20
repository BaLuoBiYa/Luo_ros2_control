import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_anymal_loopshaping_mpc') + "/config/rviz/demo_config.rviz"
    urdf_model_path = get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
    robot_description = ParameterValue(
                Command(["cat ", urdf_model_path]),
                value_type=str
    )
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_name',
            default_value='anymal_c'
        ),
        launch.actions.DeclareLaunchArgument(
            name='config_name',
            default_value='c_series'
        ),
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value='ocs2_anymal_description'
        ),
        launch.actions.DeclareLaunchArgument(
            name='perception_parameter_file',
            default_value=get_package_share_directory(
                'convex_plane_decomposition_ros') + '/config/parameters.yaml'
        ),
        # 新增：发布零位关节状态，供 robot_state_publisher 生成 TF
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            # arguments=[robot_description],
            parameters=[{'robot_description': robot_description}],
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file],
            parameters=[{'robot_description': robot_description}],
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_loopshaping_mpc',
            executable='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            name='ocs2_anymal_loopshaping_mpc_perceptive_demo',
            output='screen',
            parameters=[
                {
                    'config_name': launch.substitutions.LaunchConfiguration('config_name')
                },
                {
                    'forward_velocity': 0.5
                },
                {
                    'terrain_name': 'side_gap.png'
                },
                {
                    'ocs2_anymal_description': urdf_model_path
                },
                {
                    'terrain_scale': 0.35
                },
                {
                    'adaptReferenceToTerrain': True
                },
                launch.substitutions.LaunchConfiguration(
                    'perception_parameter_file')
            ]
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
