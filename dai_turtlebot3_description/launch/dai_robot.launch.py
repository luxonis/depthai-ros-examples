from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    turtlebot_state_pub = IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/dai_turtlebot3_state_publisher.launch.py']))

    turtlebot_node = Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen')
    
    dynamic_tracker = Node(
            package='dai_turtlebot3_description', 
            executable='dynamic_tracker',
            output='screen')

    """ turtlebot_rviz = IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot_rviz.launch.py'])) """
    """     
    depth_to_scan = Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[{'output_frame': output_frame},
                        {'range_min': range_min},
                        {'range_max': range_max}],
            arguments=['depth:=/stereo/depth',
                       'depth_camera_info:=/stereo/camera_info',
                       'scan:=/scan']) """

    ld = LaunchDescription()

    ld.add_action(turtlebot_state_pub)
    ld.add_action(turtlebot_node)
    ld.add_action(dynamic_tracker)
#     ld.add_action(turtlebot_rviz) 
    return ld

