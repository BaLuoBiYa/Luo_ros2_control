# 导入库
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

    declare_rviz = DeclareLaunchArgument(
        name = 'rviz_config',
        default_value=get_package_share_directory("legged_bringup") + "/rviz/legged_robot.rviz"
    )
    declare_xacro = DeclareLaunchArgument(
        name = 'xacro_path',
        default_value=get_package_share_directory("legged_bringup") + "/resource/anymal_c/urdf/gazebo.xacro"
    )
    declare_bridge_cfg = DeclareLaunchArgument(
        name = "bridge_config_file",
        default_value=get_package_share_directory("legged_bringup") + "/launch/gazebo_bridge.yaml"
    )
    declare_world = DeclareLaunchArgument(
        name ="world_file",
        default_value="/home/luo/Project/Horse/Luo_ros2_control/src/simulation/world.sdf"
    )
    declare_controller = DeclareLaunchArgument(
        name = 'controller_config',
        default_value=get_package_share_directory("legged_bringup") + "/launch/legged_controller.yaml"
    )
    declare_taskFile = DeclareLaunchArgument(
        name = 'taskFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/anymal_c/config/mpc/task.info"
    )
    declare_referenceFile = DeclareLaunchArgument(
        name = 'referenceFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/anymal_c/config/command/reference.info"
    )
    declare_urdfFile = DeclareLaunchArgument(
        name = 'urdfFile',
        default_value=get_package_share_directory("legged_bringup") + "/resource/anymal_c/urdf/anymal.urdf"
    )
    declare_gait_config = DeclareLaunchArgument(
        name = 'gait_config',
        default_value=get_package_share_directory("legged_bringup") + "/resource/anymal_c/config/command/gait.info"
    )

    rviz_cfg = LaunchConfiguration("rviz_config")
    xacro_path = LaunchConfiguration("xacro_path") 
    bridge_cfg = LaunchConfiguration("bridge_config_file")
    world_file = LaunchConfiguration("world_file")
    controller_cfg = LaunchConfiguration("controller_config")
    taskFile = LaunchConfiguration("taskFile")
    referenceFile = LaunchConfiguration("referenceFile")
    urdfFile = LaunchConfiguration("urdfFile")
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
                    {"use_sim_time": True},],
        remappings=[
            ("joint_states", "anymal/joint/states"),
        ],
        output="screen"
        )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        )
    
    # mpc_node = Node(
    #     package='ocs2_legged_robot_ros',
    #     executable='legged_robot_ddp_mpc',
    #     name='legged_robot_ddp_mpc',
    #     output='screen',
    #     # prefix=prefix,
    #     parameters=[
    #         {'taskFile': taskFile},
    #         {'referenceFile': referenceFile},
    #         {'urdfFile': urdfFile},
    #         {'use_sim_time': True},]
    #     )
    
    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_cfg],
        output="screen",
        # prefix=prefix,
        )
    
    # controler_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["legged_controller"],
    # )

    motor_tester_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_tester"],
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster" ,
                   "--controller-ros-args",
                   "--ros-args -r /imu_sensor_broadcaster/imu:=test_tools/imu_tester/imu",
                   ],
    )

    contact_tester_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["contact_tester"],
    )

    # spawn_delay = TimerAction(
    #     period=5.0,
    #     actions=[controler_spawner],
    # )
    
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        # prefix=prefix,
    )

    # gait_node = Node(
    #         package='ocs2_legged_robot_ros',
    #         executable='legged_robot_gait_command',
    #         name='legged_robot_gait_command',
    #         output='screen',
    #         prefix=prefix,
    #         parameters=[{'gaitCommandFile': gait_command_cfg}],
    #     )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([declare_rviz, 
                                            declare_xacro, 
                                            declare_bridge_cfg,
                                            declare_world,
                                            declare_controller,
                                            declare_taskFile,
                                            declare_referenceFile,
                                            declare_urdfFile,
                                            declare_gait_config,
                                            # gait_node,
                                            gz_sim,
                                            bridge_node,
                                            statepub_node,
                                            # rviz_node,
                                            # mpc_node,
                                            manager_node,
                                            # spawn_delay,
                                            motor_tester_spawner,
                                            imu_sensor_broadcaster_spawner,
                                            contact_tester_spawner,
                                            ])
    # 返回让ROS2根据launch描述执行节点
    return launch_description