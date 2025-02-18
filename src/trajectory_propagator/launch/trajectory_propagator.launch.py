from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("trajectory_propagator")
    config_dir = os.path.join(share_dir, "config")

    argz = []
    nodez = []

    publish_prediction = DeclareLaunchArgument(
        "publish_prediction",
        default_value="true",
        description="Whether or not to publish predictions",
    )

    publish_topic_name = DeclareLaunchArgument(
        "publish_topic_name",
        default_value="/prediction/trajectory_propagation",
        description="The topic to publish predictions on",
    )

    argz.append(publish_prediction)
    argz.append(publish_topic_name)
    nodez.append(
        Node(
            package="trajectory_propagator",
            executable="trajectory_propagator_node",
            name="trajectory_propagator",
            parameters=[
                os.path.join(config_dir, "params.yaml"),
                {publish_prediction.name: LaunchConfiguration(publish_prediction.name)},
                {publish_topic_name.name: LaunchConfiguration(publish_topic_name.name)},
            ],
            output="screen",
        )
    )

    return LaunchDescription(argz + nodez)
