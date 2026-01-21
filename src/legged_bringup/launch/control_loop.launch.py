# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    declare_controller = DeclareLaunchArgument(
        name = 'controller_config',
        default_value=get_package_share_directory("legged_bringup") + "/launch/legged_controller.yaml"
    )

    controller_cfg = LaunchConfiguration("controller_config")

    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_cfg],
        output="screen",
        # prefix=prefix,
        )
    
    controler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legged_controller"],
    )

    spawn_delay = TimerAction(
        period=0.0,
        actions=[controler_spawner],
    )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([declare_controller,
                                            manager_node,
                                            spawn_delay,
                                            ])
    # 返回让ROS2根据launch描述执行节点
    return launch_description