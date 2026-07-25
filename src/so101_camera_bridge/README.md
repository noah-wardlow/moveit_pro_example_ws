# SO-101 camera bridge

This package reads the Pi's existing MediaMTX streams and republishes them as
ROS 2 images. It never opens `/dev/video*`, changes a Pi service, or sends a
robot command.

| Physical stream | ROS image topic | Frame |
| --- | --- | --- |
| `head` | `/so101/cameras/overhead/image_raw` | `overhead_cam_optical_frame` |
| `gripper` | `/so101/cameras/wrist/image_raw` | `right_wrist_cam_optical_frame` |

The topic contract is identical to the MuJoCo configuration. Override
`SO101_HEAD_RTSP_URL` and `SO101_GRIPPER_RTSP_URL` if the Pi address changes.
The generated `CameraInfo` is an approximate pinhole model until a physical
camera calibration is available.
