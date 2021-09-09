import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from  launch_ros.descriptions import ComposableNode

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    print("Param file prsln ---------->")
    print(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    print(nav2_launch_file_dir)


    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    pointCloud_converter = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz',

                    remappings=[('image_rect', '/stereo/converted_depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',
        )

    pcl_to_scan_cmd = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[{'target_frame': "base_scan"},
                        {'angle_min': -0.7},
                        {'angle_max': 0.7},
                        {'angle_increment': 0.007},

                        {'range_min' : 0.7},
                        {'range_max' : 5.2},
                        {'min_height': 0.5},
                        {'max_height': 5.5}],
            remappings=[('cloud_in','/stereo/points')])


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        # pointCloud_converter,
        # pcl_to_scan_cmd
    ])