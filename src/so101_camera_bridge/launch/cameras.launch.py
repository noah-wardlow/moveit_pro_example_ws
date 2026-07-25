import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
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
                            "rtsp://100.79.11.87:8554/head",
                        ),
                        "gripper_url": os.environ.get(
                            "SO101_GRIPPER_RTSP_URL",
                            "rtsp://100.79.11.87:8554/gripper",
                        ),
                    }
                ],
            )
        ]
    )
