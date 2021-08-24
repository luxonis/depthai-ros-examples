from launch_ros.actions import Node

from launch import LaunchDescription, substitutions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    print(ThisLaunchFileDir())
    # dai_launch_dir = ThisLaunchFileDir()

    output_frame = substitutions.LaunchConfiguration('output_frame', default='base_scan')
    range_max = substitutions.LaunchConfiguration('range_max', default='2.0')
    range_min = substitutions.LaunchConfiguration('range_min', default='0.2')

    # lg = LogInfo(msg=[
    #         'Including launch file located at: ', ThisLaunchFileDir(), '/dai_robot.launch.py'])

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    param_file_name = TURTLEBOT3_MODEL + '.yaml'

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            param_file_name))

    #TODO(Sachin): Do I need to remap this ?
    """ remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] """
    
    turtlebot_state_pub = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
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
                       'scan:=/scan'])

    ld = LaunchDescription()
    # Declare the launch options
    # ld.add_action(lg)
    ld.add_action(turtlebot_state_pub)

    ld.add_action(turtlebot_node)
    ld.add_action(dynamic_tracker)

    return ld

