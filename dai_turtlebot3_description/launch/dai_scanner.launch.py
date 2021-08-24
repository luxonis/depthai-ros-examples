from launch_ros.actions import Node

from launch import LaunchDescription, substitutions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo

def generate_launch_description():
    print(ThisLaunchFileDir())
    # dai_launch_dir = ThisLaunchFileDir()

    output_frame = substitutions.LaunchConfiguration('output_frame', default='base_scan')
    range_max = substitutions.LaunchConfiguration('range_max', default='2.0')
    range_min = substitutions.LaunchConfiguration('range_min', default='0.2')

    # lg = LogInfo(msg=[
    #         'Including launch file located at: ', ThisLaunchFileDir(), '/dai_robot.launch.py'])

    #TODO(Sachin): Do I need to remap this ?
    """ remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')] """

    dynamic_tracker = Node(
            package='dai_turtlebot3_description', 
            executable='dynamic_tracker',
            output='screen')

    depth_to_scan = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[{'output_frame': output_frame},
                        {'range_min': range_min},
                        {'range_max': range_max}],
            remappings=[('depth','/stereo/depth'),
                        ('depth_camera_info', '/stereo/camera_info')])

    ld = LaunchDescription()
    # Declare the launch options
    # ld.add_action(lg)

    ld.add_action(dynamic_tracker)
    ld.add_action(depth_to_scan)
    return ld

