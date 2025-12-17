import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="zlac_robot",
            description="Name of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("zlac8015d_serial"),
                "rviz",
                "zlac_robot_view.rviz"
            ]),
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz if true",
        )
    )
    
    # 获取包路径
    zlac_description_dir = get_package_share_directory("zlac8015d_serial")
    
    # URDF文件路径
    urdf_file = os.path.join(zlac_description_dir, "urdf", "zlac_robot.urdf.xacro")
    
    # Robot state publisher节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    urdf_file
                ])
            }
        ],
        output="screen",
    )
    
    # Joint state publisher节点 (用于在没有实际硬件时发布关节状态)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rate": 30.0
            }
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    
    # RViz2节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )
    
    # ZLAC驱动节点
    zlac_driver_node = Node(
        package="zlac8015d_serial",
        executable="zlac_run",
        name="zlac8015d_node",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
        output="screen",
        # 可以在这里添加设备映射参数
        # remappings=[
        #     ("cmd_vel", "cmd_vel"),
        #     ("odom", "odom"),
        #     ("joint_states", "joint_states")
        # ]
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        zlac_driver_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start)