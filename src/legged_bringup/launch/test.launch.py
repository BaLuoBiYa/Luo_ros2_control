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
        default_value="world.sdf"
    )
    declare_controller = DeclareLaunchArgument(
        name = 'controller_config',
        default_value=get_package_share_directory("legged_bringup") + "/launch/legged_controller.yaml"
    )

    rviz_cfg = LaunchConfiguration("rviz_config")
    xacro_path = LaunchConfiguration("xacro_path") 
    bridge_cfg = LaunchConfiguration("bridge_config_file")
    world_file = LaunchConfiguration("world_file")
    controller_cfg = LaunchConfiguration("controller_config")

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
    
    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_cfg],
        output="screen",
        prefix=prefix,
        )

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
    
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        # prefix=prefix,
    )
    
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([declare_rviz, 
                                            declare_xacro, 
                                            declare_bridge_cfg,
                                            declare_world,
                                            declare_controller,
                                            gz_sim,
                                            bridge_node,
                                            statepub_node,
                                            rviz_node,
                                            manager_node,
                                            motor_tester_spawner,
                                            imu_sensor_broadcaster_spawner,
                                            contact_tester_spawner,
                                            ])
    # 返回让ROS2根据launch描述执行节点
    return launch_description