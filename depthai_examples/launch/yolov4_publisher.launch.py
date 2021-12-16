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
    

    camera_model     = LaunchConfiguration('camera_model',  default = 'OAK-D')
    camera_name      = LaunchConfiguration('camera_name',   default = 'oak')
    base_frame       = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame     = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    
    cam_pos_x        = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y        = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z        = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll         = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch        = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw          = LaunchConfiguration('cam_yaw',       default = '0.0')

    camera_param_uri = LaunchConfiguration('camera_param_uri', default = 'package://depthai_examples/params/camera')
    sync_nn          = LaunchConfiguration('sync_nn', default = True)
    subpixel         = LaunchConfiguration('subpixel', default = True)
    nn_path          = LaunchConfiguration('nn_path', default = "")
    confidence       = LaunchConfiguration('confidence', default = 200)
    LRchecktresh     = LaunchConfiguration('LRchecktresh', default = 5)

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=camera_name,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

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
    
    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')
    
    urdf_launch = IncludeLaunchDescription(
                            launch_description_sources.PythonLaunchDescriptionSource(
                                    os.path.join(urdf_launch_dir, 'urdf_launch.py')),
                            launch_arguments={'camera_name' : camera_name,
                                              'camera_model': camera_model,
                                              'base_frame'  : base_frame,
                                              'parent_frame': parent_frame,
                                              'cam_pos_x'   : cam_pos_x,
                                              'cam_pos_y'   : cam_pos_y,
                                              'cam_pos_z'   : cam_pos_z,
                                              'cam_roll'    : cam_roll,
                                              'cam_pitch'   : cam_pitch,
                                              'cam_yaw'     : cam_yaw}.items())

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
    
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(declare_camera_param_uri_cmd)
    ld.add_action(declare_sync_nn_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_nn_path_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)

    ld.add_action(yolov4_spatial_node)
    ld.add_action(urdf_launch)

    return ld

