from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('dai_turtlebot3_description')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    # urdf = open(urdf_path).read()
    # doc = xacro.process_file(urdf_path, mappings={'simulate_obstacles' : 'false'})
    print(xacro_path)
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    # name='tb_wafflle_description',

    rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(
                ['xacro', ' ', xacro_path])},
                {'use_sim_time': 'false'}],
                remappings = remappings)
    
    ld = LaunchDescription()

    ld.add_action(rsp_node)
    return ld
