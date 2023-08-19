import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    t265_base_frame_id = LaunchConfiguration('base_frame_id', default='odom')
    t265_serial_no = LaunchConfiguration('serial_no', default='925122110087')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'ar_ray_robot.urdf.xml'
    config_file_name = 'ar_ray_robot.rviz'

    rviz_config = os.path.join(
        get_package_share_directory('simple_mecanum_wheel'),
        config_file_name)

    urdf = os.path.join(
        get_package_share_directory('simple_mecanum_wheel'),
        urdf_file_name)

    t265_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
        output='screen',
        remappings=[('/t265/camera/odom/sample','/odom')],
        parameters=[{'serial_no':t265_serial_no ,
                'base_frame_id': t265_base_frame_id}]
        )
    robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        )

    # state_robot = Node(
    #         package='urdf_tutorial',
    #         executable='state_publisher',
    #         name='state_publisher',
    #         output='screen'
    #     )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        t265_node,
        robot_state_pub,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
        # state_robot,
  ])