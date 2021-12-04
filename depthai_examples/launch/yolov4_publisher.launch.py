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
                                'rviz', 'pointCloud.rviz')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_bridge'), 'launch')
    

    camera_name      = LaunchConfiguration('camera_name', default = 'oak')
    camera_model     = LaunchConfiguration('camera_model', default = 'OAK-D')
    camera_param_uri = LaunchConfiguration('camera_param_uri', default = 'package://depthai_examples/params/camera')
    sync_nn          = LaunchConfiguration('sync_nn', default = True)
    subpixel          = LaunchConfiguration('subpixel', default = True)
    nn_path          = LaunchConfiguration('nn_path', default = "")


    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=camera_name,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')
    
    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_camera_param_uri_cmd = DeclareLaunchArgument(
        'camera_param_uri',
        default_value=camera_param_uri,
        description='Sending camera yaml path')

    declare_sync_nn_cmd = DeclareLaunchArgument(
        'sync_nn',
        default_value=sync_nn,
        description='Syncs the image output with the Detection.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='Enables subpixel stereo detection.')

    declare_nn_path_cmd = DeclareLaunchArgument(
        'nn_path',
        default_value=nn_path,
        description='Path to the object detection blob needed for detection')
    
    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'camera_name': camera_name,
                                              'camera_model': camera_model}.items())


    yolov4_spatial_node = launch_ros.actions.Node(
            package='depthai_examples', executable='yolov4_spatial_node',
            output='screen',
            parameters=[{'camera_name': camera_name},
                        {'camera_param_uri': camera_param_uri},
                        {'sync_nn': sync_nn},
                        {'nn_path': nn_path}])

    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz])

    ld = LaunchDescription()
    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_camera_param_uri_cmd)
    ld.add_action(declare_sync_nn_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_nn_path_cmd)

    ld.add_action(yolov4_spatial_node)
    ld.add_action(urdf_launch)

    # ld.add_action(metric_converter_node)
    # ld.add_action(point_cloud_node)
    # ld.add_action(rviz_node)
    return ld

