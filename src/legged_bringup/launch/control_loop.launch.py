# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    prefix = "gnome-terminal --"
    useSimTime = False
    declare_controller = DeclareLaunchArgument(
        name = 'controller_config',
        default_value=get_package_share_directory("legged_bringup") + "/launch/legged_controller.yaml"
    )
    declare_taskFile = DeclareLaunchArgument(
        name = 'taskFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/task.info"
    )
    declare_referenceFile = DeclareLaunchArgument(
        name = 'referenceFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/reference.info"
    )
    declare_urdfFile = DeclareLaunchArgument(
        name = 'urdfFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/urdf/A1.urdf"
    )

    # controller_cfg = LaunchConfiguration("controller_config")
    taskFile = LaunchConfiguration("taskFile")
    referenceFile = LaunchConfiguration("referenceFile")
    urdfFile = LaunchConfiguration("urdfFile")

    mpc_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_ddp_mpc',
        name='legged_robot_ddp_mpc',
        output='screen',
        # prefix=prefix,
        parameters=[
            {'taskFile': taskFile},
            {'referenceFile': referenceFile},
            {'urdfFile': urdfFile},
            {'use_sim_time': useSimTime},]
        )

    # manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controller_cfg],
    #     output="screen",
    #     # prefix=prefix,
    #     )
    
    controler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legged_controller"],
        prefix=prefix
    )

    spawn_delay = TimerAction(
        period=5.0,
        actions=[controler_spawner],
    )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([declare_controller,
                                            declare_taskFile,
                                            declare_referenceFile,
                                            declare_urdfFile,
                                            mpc_node,
                                            # manager_node,
                                            spawn_delay,
                                            ])
    # 返回让ROS2根据launch描述执行节点
    return launch_description