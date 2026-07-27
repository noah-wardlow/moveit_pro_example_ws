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

## Decoding is on demand

The image topics are advertised from startup, so the camera panes list them and
`CameraInfo` is always available. Decoding, however, only runs while something is
actually subscribed to an image topic, and stops again when the last subscriber
leaves.

This matters because the operator's video no longer comes through here. MoveIt Pro
relays the Pi's stream straight to the browser (`MOVEIT_WEBRTC_PASSTHROUGH`), so
decoding every frame to raw and republishing it produced images that nothing read
— measured at 198 MB and 34 threads for two cameras. Recording, perception
Behaviors, and anything else that genuinely wants the pixels still get them by
subscribing; the decoder starts within one publish tick.

`image_width` and `image_height` (default 640x480) describe the stream so
`CameraInfo` does not need a decoded frame. If the stream turns out to be a
different size, the node warns once per camera rather than silently handing out
intrinsics for the wrong image.
