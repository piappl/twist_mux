import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def getConfigFilePath(file_name):
    # There is no nice way in ROS2 to find out source path of a package like in ROS1.
    # There is symlink to config created on first trigger of "colcon build --symlink-install"
    # so there is no need to rebuild project after change in config files.
    # More details http://answers.ros.org/question/288501/ros2-equivalent-of-rospackagegetpath/

    config_file_path = get_package_share_directory("twist_mux")
    config_file_path += "/config/"
    config_file_path += file_name

    return config_file_path

def generate_launch_description():
    twist_mux_config_file_name = getConfigFilePath("twist_mux_params.yaml")

    twist_mux = launch_ros.actions.Node(
        package='twist_mux',
        node_executable='twist_mux',
        output='screen',
        parameters=[
            twist_mux_config_file_name
            ]
    )

    return launch.LaunchDescription([
        twist_mux
    ])