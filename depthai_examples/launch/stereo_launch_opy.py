from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    """ turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {"background_b": 200},
            {"background_g": 200},
            {"background_r": 200}
        ]
    ) """
    streo_node = Node(
            package='depthai_examples', executable='stereo_node',
            parameters=[
            {"background_b": 200},
            {"background_g": 200},
            {"background_r": 200}
        ])
    # ld.add_action(turtlesim_node)
    ld.add_action(streo_node)
    return ld