import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_host = os.environ.get(
        "SO101_ROBOT_HOST", "so101-pi.tail337068.ts.net"
    )
    return LaunchDescription(
        [
            Node(
                package="so101_camera_bridge",
                executable="rtsp_camera_bridge",
                name="so101_rtsp_camera_bridge",
                output="screen",
                parameters=[
                    {
                        "head_url": os.environ.get(
                            "SO101_HEAD_RTSP_URL",
                        )
                        or f"rtsp://{robot_host}:8554/head",
                        "gripper_url": os.environ.get(
                            "SO101_GRIPPER_RTSP_URL",
                        )
                        or f"rtsp://{robot_host}:8554/gripper",
                    }
                ],
            ),
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
            ),
        ]
    )
