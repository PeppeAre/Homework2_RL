import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch configuration variables
    cmd_interface = LaunchConfiguration('cmd_interface')
    ctrl = LaunchConfiguration('ctrl')
    
    # Declare the launch arguments
    declare_cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface',
        default_value='position',
        description='Command interface to use (position, velocity, effort)')
    
    declare_ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='velocity_ctrl',
        description='Controller to use (velocity_ctrl, velocity_ctrl_null, vision)')

    # Get the path to the parameters file
    params_file = os.path.join(
        get_package_share_directory('ros2_kdl_package'),
        'config',
        'kdl_node_params.yaml'
    )

    # Create the node
    kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        parameters=[
            params_file,
            {'cmd_interface': cmd_interface},
            {'ctrl': ctrl}
        ],
        output='screen'
    )


    gz_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_service_bridge',
        arguments=[
            # Sintassi: <Nome_Servizio_GZ>@<Tipo_Servizio_ROS>
            '/world/empty/set_pose' + '@' + 'ros_gz_interfaces/srv/SetEntityPose'
        ],
        remappings=[
            # Il remapping mappa il nome del topic Gazebo al nome ROS desiderato
            ('/world/empty/set_pose', '/set_marker_pose')
        ],
        output='screen'
    )


    ld = LaunchDescription()

    ld.add_action(declare_cmd_interface_arg)
    ld.add_action(declare_ctrl_arg)
    ld.add_action(kdl_node)
    ld.add_action(gz_service_bridge)

    return ld
