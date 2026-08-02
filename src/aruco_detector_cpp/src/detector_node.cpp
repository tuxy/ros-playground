#include "aruco_msgs/msg/marker_array.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// # Model notes
// To develop a model, utilise the luxonis hub for the least pain and convert to
// shaves = 6 with legacy = on. Not sure why, but the default modern version
// seems to work less well, and requires shaves <= 7 for models on the camera to
// work. Nonetheless, the performance is impressive with only aruco detection
// (fast) and maybe hsv filtering on the host-side.
//
// # Transform publisher
// Both cube detections and aruco markers broadcast dynamic transforms to /tf.
// Cube det frames: <color>_cube (e.g. red_cube)
// Aruco det frames: aruco_marker_<id>
// Parent frame is world (default) for testing. When the camera pose is
// available in tf, global positions can be determined (from SLAM/odom) when
// parent changed.

class CubeDetectorNode : public rclcpp::Node {
public:
  CubeDetectorNode() : Node("cube_aruco_detector") {
    declare_parameter("model_path", "");
    declare_parameter("parent_tf", "world");
    declare_parameter("conf_threshold", 0.6); // Use ~0.6 due to clear classes
    declare_parameter("debug", false);

    // Might not need pub class due to tf transforms.
    pub_cubes_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/oakd/detections", 10);
    pub_aruco_ = this->create_publisher<aruco_msgs::msg::MarkerArray>(
        "/oakd/markers", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    debug_ = get_parameter("debug").as_bool();
    parent_tf_ = get_parameter("parent_tf").as_string();
    conf_threshold_ = get_parameter("conf_threshold").as_double();
  }

  ~CubeDetectorNode() override {
    if (spinner_thread_.joinable()) {
      spinner_thread_.join();
    }
  }

  void setup() {
    std::string model_path = resolve_model_path();
    RCLCPP_INFO(get_logger(), "Loading model: %s", model_path.c_str());

    if (!std::filesystem::exists(model_path)) {
      RCLCPP_ERROR(get_logger(), "Model not found: %s", model_path.c_str());
      throw std::runtime_error("Model file not found");
    }

    device_ = std::make_shared<dai::Device>();
    RCLCPP_INFO(get_logger(), "Device: %s", device_->getDeviceName().c_str());

    pipeline_ = std::make_unique<dai::Pipeline>(device_);

    // Setup RGB camera for vision/NN (CAM_A)
    auto rgbCam = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_A);

    // Setup stereo cameras for depth
    auto monoLeft = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline_->create<dai::node::StereoDepth>();
    stereo->build(*monoLeft->requestOutput({640, 400}),
                  *monoRight->requestOutput({640, 400}));
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    auto nnArchive = dai::NNArchive(model_path);

    // SpatialDetectionNetwork handles depth calculations (only for cubes)
    auto detNet = pipeline_->create<dai::node::SpatialDetectionNetwork>();
    detNet->build(rgbCam, stereo, nnArchive);

    detection_queue_ = detNet->out.createOutputQueue(4, false);
    preview_queue_ = detNet->passthrough.createOutputQueue(4, false);
    depth_queue_ = detNet->passthroughDepth.createOutputQueue(4, false);

    // Camera intrinsics for Aruco host-side depth back-projection
    auto calib = device_->readCalibration();
    auto intrinsics =
        calib.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, 416, 416);
    fx_ = intrinsics[0][0];
    fy_ = intrinsics[1][1];
    cx_ = intrinsics[0][2];
    cy_ = intrinsics[1][2];
    RCLCPP_INFO(get_logger(), "Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                fx_, fy_, cx_, cy_);

    // Pre-build ArUco detector objects (reused each frame)
    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    aruco_params_ = cv::aruco::DetectorParameters::create();

    pipeline_->start();
    RCLCPP_INFO(get_logger(), "Pipeline started");

    // Background executor thread for ROS callbacks
    spinner_thread_ =
        std::thread([this]() { rclcpp::spin(shared_from_this()); });
  }

  void spin() {
    RCLCPP_INFO(get_logger(), "Spin loop starting");

    while (rclcpp::ok()) {
      auto in_det = detection_queue_->get<dai::SpatialImgDetections>();
      auto in_frame = preview_queue_->get<dai::ImgFrame>();
      if (!in_det|| !in_frame
        continue;

      auto stamp = now();
      const std::string frame_id = "oak_rgb_camera_optical_frame";

      // NN cube detections with depth
      for (auto &det : in_det->detections) {
        if (det.confidence < conf_threshold_)
          continue;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        float scale =
            (in_det->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
        pose.pose.position.x = det.spatialCoordinates.x * scale;
        pose.pose.position.y = det.spatialCoordinates.y * scale;
        pose.pose.position.z = det.spatialCoordinates.z * scale;
        pose.pose.orientation.w = 1.0;

        if (std::isfinite(pose.pose.position.z) && pose.pose.position.z > 0.0) {
          pub_cubes_->publish(pose);

          const char *frame_name =
              (static_cast<uint32_t>(det.label) < class_names.size())
                  ? class_names[det.label]
                  : "unknown class";
          publish_tf(parent_tf_, frame_name, stamp, pose.pose.position,
                     pose.pose.orientation);
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cube detected (%.2f): x=%.3f y=%.3f z=%.3f m",
                             det.confidence, pose.pose.position.x,
                             pose.pose.position.y, pose.pose.position.z);
      }

      // Host-based Aruco detection
      cv::Mat frame = in_frame->getCvFrame();

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(frame, aruco_dict_, corners, ids, aruco_params_);

      // Depth map: non-blocking get; may lag a frame behind the preview
      auto inDepth = depth_queue_->get<dai::ImgFrame>();
      cv::Mat depthMat;
      if (inDepth) {
        depthMat = inDepth->getFrame();
      }

      if (!ids.empty()) {
        aruco_msgs::msg::MarkerArray aruco_array;
        aruco_array.header.stamp = stamp;
        aruco_array.header.frame_id = frame_id;

        for (size_t i = 0; i < ids.size(); i++) {
          aruco_msgs::msg::Marker marker;
          marker.header = aruco_array.header;
          marker.id = ids[i];

          float u = (corners[i][0].x + corners[i][2].x) / 2.0f;
          float v = (corners[i][0].y + corners[i][2].y) / 2.0f;
          float z_mm = lookup_depth(depthMat, u, v);

          if (z_mm > 0) {
            marker.pose.pose.position.x = (u - cx_) * z_mm / fx_ * 0.001f;
            marker.pose.pose.position.y = (v - cy_) * z_mm / fy_ * 0.001f;
            marker.pose.pose.position.z = z_mm * 0.001f;
            marker.pose.pose.orientation.w = 1.0;

            publish_tf(parent_tf_, "aruco_marker_" + std::to_string(ids[i]),
                       stamp, marker.pose.pose.position,
                       marker.pose.pose.orientation);
          }

          // Only publish markers with valid depth
          if (z_mm > 0) {
            aruco_array.markers.push_back(marker);
          }
        }
        pub_aruco_->publish(aruco_array);
      }

      // Debug visualization
      if (debug_) {
        debug_frame(frame, in_det, ids, corners, depthMat);
      }
    }

    // Wait for the background executor thread to wind down
    if (spinner_thread_.joinable()) {
      spinner_thread_.join();
    }
  }

  void publish_tf(const std::string &parent, const std::string &child,
                  const builtin_interfaces::msg::Time &stamp,
                  const geometry_msgs::msg::Point &pos,
                  const geometry_msgs::msg::Quaternion &orient) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = pos.x;
    t.transform.translation.y = pos.y;
    t.transform.translation.z = pos.z;
    t.transform.rotation = orient;
    tf_broadcaster_->sendTransform(t);
  }

  float lookup_depth(const cv::Mat &depth, float u, float v) {
    if (depth.empty())
      return 0;
    int px = static_cast<int>(u), py = static_cast<int>(v);
    int half = 2; // 5x5 window
    uint32_t sum = 0, count = 0;
    for (int dy = -half; dy <= half; dy++) {
      for (int dx = -half; dx <= half; dx++) {
        int x = px + dx, y = py + dy;
        if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows)
          continue;
        uint16_t z = depth.at<uint16_t>(y, x);
        if (z > 0) {
          sum += z;
          count++;
        }
      }
    }
    return count > 0 ? static_cast<float>(sum) / count : 0;
  }

  void debug_frame(cv::Mat &frame,
                   const std::shared_ptr<dai::SpatialImgDetections> &in_det,
                   const std::vector<int> &ids,
                   const std::vector<std::vector<cv::Point2f>> &corners,
                   const cv::Mat &depthMat) {

    float scale = (in_det->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
    int w = frame.cols;
    int h = frame.rows;

    // Draw cube detections: bounding box + label + depth
    for (auto &det : in_det->detections) {
      int x1 = std::max(0, static_cast<int>(det.xmin * w));
      int y1 = std::max(0, static_cast<int>(det.ymin * h));
      int x2 = std::min(w, static_cast<int>(det.xmax * w));
      int y2 = std::min(h, static_cast<int>(det.ymax * h));

      const char *name = (static_cast<uint32_t>(det.label) < class_names.size())
                             ? class_names[det.label]
                             : "unknown class";
      float x = det.spatialCoordinates.x * scale;
      float y = det.spatialCoordinates.y * scale;
      float z = det.spatialCoordinates.z * scale;

      cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(0, 255, 0), 2);
      std::string label =
          std::string(name) + " " +
          std::to_string(static_cast<int>(det.confidence * 100)) +
          " % X =" + x + "m" + " % Y =" + y + "m" + " % Z =" + z + "m";
      int baseline = 0;
      cv::Size text_sz =
          cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
      cv::rectangle(frame, cv::Point(x1, y1 - text_sz.height - 4),
                    cv::Point(x1 + text_sz.width, y1), cv::Scalar(0, 255, 0),
                    cv::FILLED);
      cv::putText(frame, label, cv::Point(x1, y1 - 2), cv::FONT_HERSHEY_SIMPLEX,
                  0.4, cv::Scalar(0, 0, 0), 1);
    }

    // Draw ArUco markers + depth labels
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
      for (size_t i = 0; i < ids.size(); i++) {
        float u = (corners[i][0].x + corners[i][2].x) / 2.0f;
        float v = (corners[i][0].y + corners[i][2].y) / 2.0f;
        float z_mm = lookup_depth(depthMat, u, v);
        float z_m = z_mm * 0.001f;
        std::string label = "ID=" + std::to_string(ids[i]) + " Z=" + z_m + "m";
        cv::putText(frame, label, cv::Point(u, v - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
      }
    }

    cv::namedWindow("OAKD-LR NN Output", cv::WINDOW_NORMAL);
    cv::imshow("OAKD-LR NN Output", frame);

    cv::waitKey(1);
  }

private:
  static constexpr std::array<const char *, 5> class_names = {
      "red_cube", "green_cube", "blue_cube", "yellow_cube", "gray_cube"};

  std::string resolve_model_path() {
    auto param_path = get_parameter("model_path").as_string();
    if (!param_path.empty() && std::filesystem::exists(param_path))
      return param_path;
    std::vector<std::string> candidates = {
        "/ros_ws/src/aruco_detector_cpp/models/"
        "best_416.rvc2_legacy.rvc2.tar.xz",
        "models/best.rvc2.tar.xz",
    };
    for (auto &p : candidates) {
      if (!p.empty() && std::filesystem::exists(p))
        return p;
    }
    return param_path;
  }

  bool debug_ = false;
  float conf_threshold_ = 0.6f;
  std::string parent_tf_ = "";
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> preview_queue_;
  std::shared_ptr<dai::MessageQueue> detection_queue_;
  std::shared_ptr<dai::MessageQueue> depth_queue_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
  std::thread spinner_thread_;
  float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cubes_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr pub_aruco_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CubeDetectorNode>();
    node->setup();
    node->spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("detector_node"), "Error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
