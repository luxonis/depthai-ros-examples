import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
        default_rviz = os.path.join(get_package_share_directory('dai_turtlebot3_description'),
                                'rviz', 'turtlebot.rviz')
        # rviz
        rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])

        ld = LaunchDescription()
        ld.add_action(rviz_node)
        return ld