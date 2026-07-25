import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    user_ws = Path(
        os.environ.get("USER_WS", str(Path.home() / "user_ws"))
    ).resolve()
    runtime_dir = user_ws / ".runtime"
    return LaunchDescription(
        [
            Node(
                package="so101_zenoh_ros",
                executable="so101_zenoh_bridge",
                name="so101_zenoh_bridge",
                output="screen",
                parameters=[
                    {
                        "endpoint": os.environ.get(
                            "SO101_ZENOH_ENDPOINT",
                            "tcp/100.79.11.87:7447",
                        ),
                        "control_key_file": str(
                            runtime_dir / "control-auth.key"
                        ),
                        "commands_enabled_file": str(
                            runtime_dir / "hardware-commands-enabled"
                        ),
                    }
                ],
            ),
            Node(
                package="so101_camera_bridge",
                executable="rtsp_camera_bridge",
                name="so101_rtsp_camera_bridge",
                output="screen",
                parameters=[
                    {
                        "head_url": os.environ.get(
                            "SO101_HEAD_RTSP_URL",
                            "rtsp://100.79.11.87:8554/head",
                        ),
                        "gripper_url": os.environ.get(
                            "SO101_GRIPPER_RTSP_URL",
                            "rtsp://100.79.11.87:8554/gripper",
                        ),
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
