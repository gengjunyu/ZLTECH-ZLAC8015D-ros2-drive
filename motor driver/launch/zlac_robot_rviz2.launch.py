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
    # 包路径
    pkg_dir = get_package_share_directory('zlac8015d_serial')
    
    # 声明参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start RViz if true'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_dir, 'rviz', 'zlac_robot_view.rviz'),
            description='RViz configuration file'
        )
    )
    
    # URDF文件
    urdf_file = os.path.join(pkg_dir, 'urdf', 'zlac_robot.urdf.xacro')
    
    # 获取机器人描述
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])
    
    # 节点定义
    nodes = []
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description
            }
        ]
    )
    nodes.append(robot_state_publisher_node)
    
    # Joint State Publisher GUI (用于测试和可视化)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rate': 30.0
            }
        ]
    )
    nodes.append(joint_state_publisher_gui_node)
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    nodes.append(rviz_node)
    
    # ZLAC电机驱动节点 (可选，用于实际硬件)
    zlac_driver_node = Node(
        package='zlac8015d_serial',
        executable='zlac_run',
        name='zlac8015d_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        # 注释掉以避免在没有硬件时出错
        # remappings=[
        #     ('cmd_vel', 'cmd_vel'),
        #     ('odom', 'odom'),
        #     ('joint_states', 'joint_states')
        # ]
    )
    # nodes.append(zlac_driver_node)
    
    return LaunchDescription(declared_arguments + nodes)