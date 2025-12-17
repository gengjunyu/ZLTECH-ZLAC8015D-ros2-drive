import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 包路径
    package_dir = get_package_share_directory('zlac8015d_serial')
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([package_dir, 'urdf', 'zlac_robot.urdf.xacro'])
    ])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content
            }
        ]
    )
    
    # Joint state publisher (for visualization without hardware)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Static transform publisher for odom -> base_link
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_config_file = os.path.join(package_dir, 'rviz', 'zlac_robot_simple.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_transform_publisher_node,
        rviz_node
    ])