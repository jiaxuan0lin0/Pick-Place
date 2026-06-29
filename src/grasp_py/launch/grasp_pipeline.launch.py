from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    grasp_ws = LaunchConfiguration('grasp_ws')
    image_save_path = LaunchConfiguration('image_save_path')
    socket_host = LaunchConfiguration('socket_host')
    socket_port = LaunchConfiguration('socket_port')
    visualize_sampled = LaunchConfiguration('visualize_sampled')
    max_depth_raw = LaunchConfiguration('max_depth_raw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'grasp_ws',
            default_value=EnvironmentVariable('GRASP_WS', default_value='/data/jiaxuanLin/grasp_ws'),
            description='Workspace root used by GraspNet, Grounded-SAM and socket image paths.',
        ),
        DeclareLaunchArgument(
            'image_save_path',
            default_value=PathJoinSubstitution([grasp_ws, 'data', 'images']),
            description='Directory where socket_server saves color.png/depth.png.',
        ),
        DeclareLaunchArgument(
            'socket_host',
            default_value='0.0.0.0',
            description='Socket server bind address.',
        ),
        DeclareLaunchArgument(
            'socket_port',
            default_value='9090',
            description='Socket server TCP port.',
        ),
        DeclareLaunchArgument(
            'visualize_sampled',
            default_value='false',
            description='Whether graspnet_server opens Open3D visualization windows.',
        ),
        DeclareLaunchArgument(
            'max_depth_raw',
            default_value='2000',
            description='Maximum raw depth value used by graspnet_server; use 0 to disable the upper bound.',
        ),
        SetEnvironmentVariable('GRASP_WS', grasp_ws),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='grounded_sam_py',
            executable='grounded_sam_server',
            name='grounded_sam_server',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='graspnet_py',
            executable='graspnet_server',
            name='graspnet_server',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'visualize_sampled': ParameterValue(visualize_sampled, value_type=bool),
                'max_depth_raw': ParameterValue(max_depth_raw, value_type=int),
            }],
        ),
        Node(
            package='grasp_py',
            executable='grasp_client',
            name='grasp_client',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='socket_node',
            executable='socket_server',
            name='socket_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'socket_host': socket_host,
                'socket_port': ParameterValue(socket_port, value_type=int),
                'image_save_path': image_save_path,
            }],
        ),
    ])
