from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),  # Ora configurabile
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[('/camera_info', LaunchConfiguration('camera_info_topic')),
                    ('/image', LaunchConfiguration('image_topic'))],
    )

    return [aruco_single]

def generate_launch_description():
    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='201',  # Il tuo ID
        description='Marker ID.'
    )
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.1',  # Il tuo size
        description='Marker size in m.'
    )
    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be refered.'
    )
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='',
        description='Reference frame. Leave empty to publish wrt parent_name.'
    )
    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement.',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )
    # Nuovi args per single camera
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera',
        description='Image topic to subscribe to.'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera_info',
        description='Camera info topic to subscribe to.'
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera_optical_frame',
        description='Camera optical frame.'
    )

    ld = LaunchDescription()
    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame_arg)
    ld.add_action(corner_refinement_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
