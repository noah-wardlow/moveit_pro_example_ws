import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    default_url = os.environ.get(
        "SO101_INFERENCE_URL", "http://127.0.0.1:8973/infer"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "inference_url",
                default_value=default_url,
                description="HTTP endpoint implementing the SO-101 policy protocol.",
            ),
            DeclareLaunchArgument(
                "request_timeout_seconds",
                default_value="30.0",
            ),
            DeclareLaunchArgument("jpeg_quality", default_value="90"),
            Node(
                package="so101_vla_adapter",
                executable="get_action_chunk_adapter",
                name="so101_get_action_chunk_adapter",
                output="screen",
                parameters=[
                    {
                        "inference_url": LaunchConfiguration("inference_url"),
                        "request_timeout_seconds": ParameterValue(
                            LaunchConfiguration("request_timeout_seconds"),
                            value_type=float,
                        ),
                        "jpeg_quality": ParameterValue(
                            LaunchConfiguration("jpeg_quality"),
                            value_type=int,
                        ),
                    },
                    os.path.join(
                        os.path.dirname(os.path.dirname(__file__)),
                        "config",
                        "adapter.yaml",
                    ),
                ],
            ),
        ]
    )
