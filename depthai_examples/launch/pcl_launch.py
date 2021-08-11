import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depthai_bridge'),
                                'rviz', 'pointCloud.rviz')
    print(default_rviz)
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')
    print("Printing urdf path----------------------")
    print(urdf_launch_dir)
    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')))
    ld = LaunchDescription([

        launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_node',
            output='screen'),

            launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',

                    remappings=[('depth/image_rect', '/stereo/depth'),
                                ('intensity/image_rect', '/right/image'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',
        ),
        urdf_launch,

        # rviz
        launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz]),

    ])

    return ld



                    # remappings=[('image_rect', '/stereo/depth'),
                    #             ('camera_info', '/stereo/camera_info'),
                    #             ('image', '/stereo/converted_image')]