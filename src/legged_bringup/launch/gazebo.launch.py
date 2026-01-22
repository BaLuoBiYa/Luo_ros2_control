# 导入库
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    prefix = "gnome-terminal --"
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/jazzy/lib/"
    useSimTime = True

    declare_rviz = DeclareLaunchArgument(
        name = 'rviz_config',
        default_value=get_package_share_directory("legged_bringup") + "/rviz/legged_robot.rviz"
    )
    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/urdf/gazebo.xacro"
    )
    declare_bridge_cfg = DeclareLaunchArgument(
        name = "bridge_config_file",
        default_value=get_package_share_directory("legged_bringup") + "/launch/gazebo_bridge.yaml"
    )
    declare_world = DeclareLaunchArgument(
        name ="world_file",
        default_value="world.sdf"
    )
    declare_referenceFile = DeclareLaunchArgument(
        name = 'referenceFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/reference.info"
    )

    declare_gait_config = DeclareLaunchArgument(
        name = 'gait_config',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/gait.info"
    )

    rviz_cfg = LaunchConfiguration("rviz_config")
    xacro_path = LaunchConfiguration("xacro_path") 
    bridge_cfg = LaunchConfiguration("bridge_config_file")
    world_file = LaunchConfiguration("world_file")
    referenceFile = LaunchConfiguration("referenceFile")
    gait_command_cfg = LaunchConfiguration("gait_config")


    robot_description_content = ParameterValue(
                Command(["xacro ", xacro_path]),
                value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_cfg}],
        output="screen"
    )

    statepub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description,
                    {"use_sim_time": useSimTime},],
        # remappings=[
        #     ("joint_states", "anymal/joint/states"),
        # ],
        output="screen"
        )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{'use_sim_time':useSimTime}]
        )
    
    target_node = Node(
        package="ocs2_legged_robot_ros",
        executable="legged_robot_target",
        name='legged_robot_target',
        output='screen',
        prefix=prefix,
        parameters=[{'referenceFile': referenceFile},
                    {'use_sim_time':useSimTime}]
    )
    
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        # prefix=prefix,
        env=env,
    )

    gait_node = Node(
            package='ocs2_legged_robot_ros',
            executable='legged_robot_gait_command',
            name='legged_robot_gait_command',
            output='screen',
            prefix=prefix,
            parameters=[{'gaitCommandFile': gait_command_cfg}],
        )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([declare_rviz, 
                                            declare_xacro, 
                                            declare_bridge_cfg,
                                            declare_world,
                                            declare_gait_config,
                                            declare_referenceFile,
                                            gz_sim,
                                            gait_node,
                                            bridge_node,
                                            statepub_node,
                                            rviz_node,
                                            target_node,
                                            ])
    # 返回让ROS2根据launch描述执行节点
    return launch_description