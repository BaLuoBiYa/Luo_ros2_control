# 导入库
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    prefix = "gnome-terminal --"
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/jazzy/lib/"
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    # 桥接节点，发布/clock

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args','empty.sdf -r')]
    )
    # 包括ros2中封装的gz sim启动脚本

    gz_spawn_scene = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', 'model://scene', 
                   '-name','obstacle',
                   '-z','0.01',
                   '-R','3.141592654'],
    )
    # 生成地面障碍物

    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', 'model://A1', 
                   '-name','A1',
                   '-z','0.4'],
    )
    # 生成机器人

    statepub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen"
        )
    # 发布机器人的urdf描述，提供给rviz和ros2 control

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        )
    # 启动rviz可视化

    controler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["legged_controller"],
        parameters=[{"controller_manager_timeout": 500.0},
                    {"inactive-timeout":500.0}],
    )
    # 加载主控制器

    gait_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_gait_command',
        name='legged_robot_gait_command',
        output='screen',
        prefix=prefix,
        parameters=[{'gaitCommandFile': gait_command_cfg},
                    {'use_sim_time':True}],
    )
    # 步态发布器

    timeLine0 = TimerAction(
        period=0.0,
        actions=[gz_sim_launch,statepub_node,rviz_node,gait_node],
    )

    timeLine1 = TimerAction(
        period=3.0,
        actions=[bridge],
    )

    timeLine2 = TimerAction(
        period=6.0,
        actions=[gz_spawn_scene],
    )

    timeLine3 = TimerAction(
        period=9.0,
        actions=[gz_spawn_robot],
    )

    timeLine4 = TimerAction(
        period=12.0,
        actions=[controler_spawner],
    )
    
    ld = LaunchDescription([
            declare_rviz,
            declare_xacro,
            declare_bridge_cfg,
            declare_world,
            declare_gait_config,
            declare_referenceFile,
            timeLine0,
            timeLine1,
            timeLine2,
            timeLine3,
            timeLine4,
        ])
    
    return ld