import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')
    
    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')

    name = LaunchConfiguration('name').perform(context)

    cliff_detection_config = LaunchConfiguration("cliff_detection_params_file")

    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')
    
    return [
           Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '-0.040000', '--y', '-0.072', '--z', '-0.009', '--roll', '1.57', '--pitch', '4.71', '--yaw', '0.0','--frame-id', 'camera_rgb_optical_frame', '--child-frame-id', 'oak-d-base-frame']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': name,
                              'camera_model': camera_model,
                              'base_frame': name,
                              'parent_frame': parent_frame,
                              'cam_pos_x': cam_pos_x,
                              'cam_pos_y': cam_pos_y,
                              'cam_pos_z': cam_pos_z,
                              'cam_roll': cam_roll,
                              'cam_pitch': cam_pitch,
                              'cam_yaw': cam_yaw}.items()),

        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file],
                    ),
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::RectifyNode',
                        name='rectify_color_node',
                        remappings=[
                            ('image', name+'/left/image_raw'),
                            ('camera_info', name+'/left/camera_info'),
                            ('image_rect', name+'/left/image_rect')
                        ]
                    ),
                    ComposableNode(
                        package="depthai_filters",
                        plugin="depthai_filters::WLSFilter",
                        remappings=[('stereo/image_raw', name+'/stereo/image_raw'),
                                    ('stereo/camera_info', name+'/stereo/camera_info'),
                                    ('left/image_raw', name+'/left/image_rect')]
                    )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        ),
        Node(
        package='cliff_detector',
        executable='cliff_detector_exe',
        parameters=[cliff_detection_config],
        remappings=[
            ('/image', '/wls_filtered'),
            #('/image', '/oak/stereo/image_raw'),
            ('/camera_info', '/oak/stereo/camera_info'),
        ]
    )
        

    ]


def generate_launch_description():
    cliff_detection_prefix = get_package_share_directory("depthai_cliff_detection")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(cliff_detection_prefix, 'config', 'cliff_detection.yaml')),
        DeclareLaunchArgument("cliff_detection_params_file", default_value=os.path.join(cliff_detection_prefix, 'config', 'params.yaml')),
        DeclareLaunchArgument("use_rviz", default_value='false'),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(cliff_detection_prefix, "config", "rviz", "rgbd.rviz"))
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )



