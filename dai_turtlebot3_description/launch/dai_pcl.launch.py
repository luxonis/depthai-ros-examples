from launch_ros.actions import Node, ComposableNodeContainer

from launch import LaunchDescription, substitutions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo
import launch_ros.descriptions

def generate_launch_description():
    print(ThisLaunchFileDir())
    # dai_launch_dir = ThisLaunchFileDir()
    default_rviz = os.path.join(get_package_share_directory('dai_turtlebot3_description'),
                                'rviz', 'tbot_depth_scan.rviz')
    output_frame = substitutions.LaunchConfiguration('output_frame', default='base_scan')
    range_max = substitutions.LaunchConfiguration('range_max', default='8.0')
    range_min = substitutions.LaunchConfiguration('range_min', default='0.8')

    # lg = LogInfo(msg=[
    #         'Including launch file located at: ', ThisLaunchFileDir(), '/dai_robot.launch.py'])

    #TODO(Sachin): Do I need to remap this ?
    """ remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] """
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')
    urdf_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')))
  
    dynamic_tracker = Node(
            package='dai_turtlebot3_description', 
            executable='dynamic_tracker',
            output='screen')


    
    metric_converter = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
            ],
            output='screen',
        )

    depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            # prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[{'output_frame': 'oak-d_right_camera_frame'},
                        {'range_min': range_min},
                        {'range_max': range_max},
                        {'scan_height': 5}],
            remappings=[('depth','/stereo/depth'),
                        ('depth_camera_info', '/stereo/camera_info')])

    pointCloud_converter = ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::ConvertMetricNode',
                    name='convert_metric_node',
                    remappings=[('image_raw', '/stereo/depth'),
                                ('camera_info', '/stereo/camera_info'),
                                ('image', '/stereo/converted_depth')]
                ),
                launch_ros.descriptions.ComposableNode(
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

    pcl_to_scan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[{'target_frame': "base_scan"},
                        {'angle_min': -0.7},
                        {'angle_max': 0.7},
                        {'angle_increment': 0.002},
                        
                        {'range_min' : 0.7},
                        {'range_max' : 5.2},
                        {'min_height': 0.5},
                        {'max_height': 5.5}],
            remappings=[('cloud_in','/stereo/points')])

    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])
    ld = LaunchDescription()
    # Declare the launch options
    # ld.add_action(lg)
    # ld.add_action(urdf_launch)
    # ld.add_action(dynamic_tracker)
    # ld.add_action(pointCloud_converter)
    # ld.add_action(pcl_to_scan)

    # ld.add_action(metric_converter)
    ld.add_action(depth_to_scan)

    ld.add_action(rviz_node)
    # ld.add_action(depth_to_scan)
    return ld

