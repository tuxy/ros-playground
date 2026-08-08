# Parameters

| Parameter        | Type        | Default                                                    | Description |
|------------------|-------------|------------------------------------------------------------|-------------|
| `model_path`     | string      | `""` (empty)                                               | Path to the YOLO `.rvc2` model archive. Searches param dir before using the share dir. |
| `conf_threshold` | double      | `0.6`                                                      | Minimum confidence for a cube detection to be published. |
| `rgb_fps`        | double      | `15.0`                                                     | Throttles the RGB camera feed fed into the NN. Lower = less CPU, higher = fresher detections. |
| `class_names`    | string[]    | `["red_cube", "green_cube", "blue_cube", "yellow_cube", "gray_cube"]` | Class labels for the NN output, indexed by detection label. Used for the TF child frame name and the debug overlay. |
| `parent_tf`       | string | `"world"`                            | Parent frame for all published TF transforms. Change to camera frame when camera pose is available. |
| `frame_id`        | string | `"oak_rgb_camera_optical_frame"`     | Frame attached to published PoseStamped / MarkerArray headers. |
| `topic_namespace` | string | `"/oakd"`                            | ROS2 Topic namespace to use |
| `debug`           | bool   | `false`                              | Opens an OpenCV debug window and draws cube boxes + marker overlays. Disable for headless runs. |

## Output topics
- `<topic_namespace>/detections`: `geometry_msgs/msg/PoseStamped`
- `<topic_namespace>/markers`: `aruco_msgs/msg/MarkerArray`

## TF frames
- `<color>_cube`       (e.g. `red_cube`) — child of `parent_tf`, NN detections
- `aruco_marker_<id>`    — child of `parent_tf`, ArUco detections, should also be child of `parent_tf`
