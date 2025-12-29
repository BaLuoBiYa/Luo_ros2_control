from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'legged_description/anymal_c') + '/config/mpc/task.info'
        ),
        DeclareLaunchArgument(
            name='referenceFile',
            default_value=get_package_share_directory(
                'legged_description/anymal_c') + '/config/command/reference.info'
        ),
        DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'legged_description/anymal_c') + '/urdf/anymal.urdf'
        ),

        Node(
            package='ocs2_legged_robot_ros',
            executable='legged_robot_ddp_mpc',
            name='legged_robot_ddp_mpc',
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'referenceFile': LaunchConfiguration('referenceFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
            ]
        ),
    ])
