from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("beam_estimator")
    config_path = os.path.join(package_share, "config", "beam_estimator.yaml")
    return LaunchDescription(
        [
            Node(
                package="beam_estimator",
                executable="beam_estimator_node",
                name="beam_estimator",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
