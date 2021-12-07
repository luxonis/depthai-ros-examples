import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depthai_examples'),
                                'rviz', 'stereoInertial.rviz')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')
    

    camera_name  = LaunchConfiguration('camera_name', default = 'oak')
    camera_model = LaunchConfiguration('camera_model', default = 'OAK-D')
    mode         = LaunchConfiguration('mode', default = 'depth')
    lrcheck      = LaunchConfiguration('lrcheck', default = True)
    extended     = LaunchConfiguration('extended', default = False)
    subpixel     = LaunchConfiguration('subpixel', default = True)

    rectify        = LaunchConfiguration('rectify', default = True)
    depth_aligned  = LaunchConfiguration('depth_aligned', default = False)
    stereo_fps     = LaunchConfiguration('stereo_fps', default = 30)
    confidence     = LaunchConfiguration('confidence', default = 200)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 5)

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=camera_name,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')
    
    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value=mode,
        description='set to depth or disparity. Setting to depth will publish depth or else will publish disparity.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_rectify_cmd = DeclareLaunchArgument(
        'rectify',
        default_value=rectify,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_depth_aligned_cmd = DeclareLaunchArgument(
        'depth_aligned',
        default_value=depth_aligned,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_stereo_fps_cmd = DeclareLaunchArgument(
        'stereo_fps',
        default_value=stereo_fps,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    
    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'camera_name': camera_name,
                                              'camera_model': camera_model}.items())


    streo_node = launch_ros.actions.Node(
            package='depthai_examples', executable='stereo_inertial_node',
            output='screen',
            parameters=[{'camera_name':   camera_name},
                        {'mode':          mode},
                        {'lrcheck':       lrcheck},
                        {'extended':      extended},
                        {'subpixel':      subpixel},
                        {'rectify':       rectify},
                        {'depth_aligned': depth_aligned},
                        {'stereo_fps':    stereo_fps},
                        {'confidence':    confidence},
                        {'LRchecktresh':  LRchecktresh}])

    metric_converter_node = launch_ros.actions.ComposableNodeContainer(
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
            output='screen',)

    point_cloud_node = launch_ros.actions.ComposableNodeContainer(
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

                    remappings=[('depth/image_rect', '/stereo/converted_depth'),
                                ('intensity/image_rect', '/right/image_rect'),
                                ('intensity/camera_info', '/right/camera_info'),
                                ('points', '/stereo/points')]
                ),
            ],
            output='screen',)

    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])

    ld = LaunchDescription()
    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_rectify_cmd)
    ld.add_action(declare_depth_aligned_cmd)
    ld.add_action(declare_stereo_fps_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_confidence_cmd)

    ld.add_action(streo_node)
    ld.add_action(urdf_launch)

    ld.add_action(metric_converter_node)
    ld.add_action(point_cloud_node)
    ld.add_action(rviz_node)
    return ld

