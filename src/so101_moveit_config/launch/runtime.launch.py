import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="so101_vla_adapter",
                executable="get_action_chunk_adapter",
                name="so101_get_action_chunk_adapter",
                output="screen",
                parameters=[
                    {
                        "inference_url": os.environ.get(
                            "SO101_INFERENCE_URL",
                            "http://127.0.0.1:8973/infer",
                        )
                    }
                ],
            )
        ]
    )
