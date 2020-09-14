import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    twist_marker = launch_ros.actions.Node(
        package='twist_mux',
        node_executable='twist_marker',
        output='screen'
    )

    tf_static_transformer = launch_ros.actions.Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        output='screen',
        arguments=[
            '1', '1', '1', '0.0', '0.0', '0.0', 'world', 'base_link'
        ]
    )

    return launch.LaunchDescription([
        twist_marker,
        tf_static_transformer
    ])