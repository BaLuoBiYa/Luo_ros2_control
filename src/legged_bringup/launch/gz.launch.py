# 导入库
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,TimerAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    prefix = prefix = prefix = "xterm -hold -e"
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"/opt/ros/jazzy/lib/"
    env["GZ_SIM_RESOURCE_PATH"] = get_package_share_directory("legged_sim")

    declare_rviz = DeclareLaunchArgument(
        name = 'rviz_config',
        default_value=get_package_share_directory("legged_bringup") + "/rviz/legged_robot.rviz"
    )
    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/urdf/gazebo.xacro"
    )

    declare_urdfFile = DeclareLaunchArgument(
        name = 'urdfFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/urdf/A1.urdf"
    )
    declare_referenceFile = DeclareLaunchArgument(
        name = 'referenceFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/reference.info"
    )
    declare_taskFile = DeclareLaunchArgument(
        name = 'taskFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/task.info"
    )
    declare_gaitFile = DeclareLaunchArgument(
        name = 'gaitFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/A1/config/gait.info"
    )

    rviz_cfg = LaunchConfiguration("rviz_config")
    xacro_path = LaunchConfiguration("xacro_path")

    urdfFile =  LaunchConfiguration("urdfFile")
    referenceFile = LaunchConfiguration("referenceFile")
    taskFile =  LaunchConfiguration("taskFile")
    gaitFile = LaunchConfiguration("gaitFile")

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
        env = env,
    )
    # 生成地面障碍物

    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', 'model://A1', 
                   '-name','A1',
                   '-z','0.4'],
        env = env,
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
    )
    # 加载主控制器

    gait_node = Node(
        package='ocs2_legged_robot_ros',
        executable='legged_robot_gait_command',
        name='legged_robot_gait_command',
        output='screen',
        prefix=prefix,
        parameters=[{'gaitCommandFile': gaitFile},
                    {'use_sim_time':True}],
    )
    # 步态发布器

    target_node = Node(
        package="ocs2_legged_robot_ros",
        executable="legged_robot_target",
        name='legged_robot_target',
        output='screen',
        prefix=prefix,
        parameters=[{'referenceFile': referenceFile},
                    {'use_sim_time':True}]
    )
    # 目标发布器

    mpc_node = Node(
            package='legged_controllers',
            executable='legged_robot_sqp_mpc',
            name='legged_robot_sqp_mpc',
            output='screen',
            prefix= prefix,
            parameters=[{'multiplot': False},
                {'taskFile': taskFile},
                {'referenceFile': referenceFile},
                {'urdfFile': urdfFile}
            ]
    )

    timeLine0 = TimerAction(
        period=0.0,
        actions=[gz_sim_launch,statepub_node,rviz_node,gait_node,target_node,mpc_node],
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
        period=15.0,
        actions=[controler_spawner],
    )
    
    ld = LaunchDescription([
            declare_rviz,
            declare_xacro,
            declare_urdfFile,
            declare_referenceFile,
            declare_taskFile,
            declare_gaitFile,
            timeLine0,
            timeLine1,
            timeLine2,
            timeLine3,
            timeLine4,
        ])
    
    return ld