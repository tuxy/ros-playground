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
#include <filesystem>
#include <memory>
#include <string>
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
// Cube det frames: cube_<color>
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

    qDet_ = detNet->out.createOutputQueue(4, false);
    qPreview_ = detNet->passthrough.createOutputQueue(4, false);
    qDepth_ = detNet->passthroughDepth.createOutputQueue(4, false);

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

    pipeline_->start();
    RCLCPP_INFO(get_logger(), "Pipeline started");
  }

  void spin() {
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    auto params = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(get_logger(), "Spin loop starting");

    while (rclcpp::ok()) {
      auto inDet = qDet_->get<dai::SpatialImgDetections>();
      auto inFrame = qPreview_->get<dai::ImgFrame>();
      if (!inDet || !inFrame)
        continue;

      auto stamp = now();
      const std::string frame_id = "oak_rgb_camera_optical_frame";

      // NN cube detections with depth
      for (auto &det : inDet->detections) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        float scale =
            (inDet->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
        pose.pose.position.x = det.spatialCoordinates.x * scale;
        pose.pose.position.y = det.spatialCoordinates.y * scale;
        pose.pose.position.z = det.spatialCoordinates.z * scale;
        pose.pose.orientation.w = 1.0;

        if (pose.pose.position.z != 0.0) {
          pub_cubes_->publish(pose);

          const std::array<const char *, 5> class_names = {
              "red", "green", "blue", "yellow", "gray"};
          const char *color = (det.label < class_names.size())
                                  ? class_names[det.label]
                                  : "unknown";
          publish_tf(parent_tf_, std::string("cube_") + color, stamp,
                     pose.pose.position, pose.pose.orientation);
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cube detected (%.2f): x=%.3f y=%.3f z=%.3f m",
                             det.confidence, pose.pose.position.x,
                             pose.pose.position.y, pose.pose.position.z);
      }

      // Host-based Aruco detection
      cv::Mat frame = inFrame->getCvFrame();

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(frame, dict, corners, ids, params);

      // Depth map: use blocking get to ensure we have a frame
      // passthroughDepth may be at a different resolution than RGB
      auto inDepth = qDepth_->get<dai::ImgFrame>();
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

          aruco_array.markers.push_back(marker);
        }
        pub_aruco_->publish(aruco_array);
      }

      // Debug visualization
      if (debug_) {
        debug_frame(frame, inDet, ids, corners, depthMat);
      }

      rclcpp::spin_some(shared_from_this());
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
                   const std::shared_ptr<dai::SpatialImgDetections> &inDet,
                   const std::vector<int> &ids,
                   const std::vector<std::vector<cv::Point2f>> &corners,
                   const cv::Mat &depthMat) {
    const std::array<const char *, 5> class_names = {
        "red_cube", "green_cube", "blue_cube", "yellow_cube", "gray_cube"};

    // FOR YOLO MODEL TESTING
    // std::array<const char *, 80> class_names = {
    //     "person",        "bicycle",      "car",
    //     "motorcycle",    "airplane",     "bus",
    //     "train",         "truck",        "boat",
    //     "traffic light", "fire hydrant", "stop sign",
    //     "parking meter", "bench",        "bird",
    //     "cat",           "dog",          "horse",
    //     "sheep",         "cow",          "elephant",
    //     "bear",          "zebra",        "giraffe",
    //     "backpack",      "umbrella",     "handbag",
    //     "tie",           "suitcase",     "frisbee",
    //     "skis",          "snowboard",    "sports ball",
    //     "kite",          "baseball bat", "baseball glove",
    //     "skateboard",    "surfboard",    "tennis racket",
    //     "bottle",        "wine glass",   "cup",
    //     "fork",          "knife",        "spoon",
    //     "bowl",          "banana",       "apple",
    //     "sandwich",      "orange",       "broccoli",
    //     "carrot",        "hot dog",      "pizza",
    //     "donut",         "cake",         "chair",
    //     "couch",         "potted plant", "bed",
    //     "dining table",  "toilet",       "tv",
    //     "laptop",        "mouse",        "remote",
    //     "keyboard",      "cell phone",   "microwave",
    //     "oven",          "toaster",      "sink",
    //     "refrigerator",  "book",         "clock",
    //     "vase",          "scissors",     "teddy bear",
    //     "hair drier",    "toothbrush"};

    float scale = (inDet->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
    int w = frame.cols;
    int h = frame.rows;

    // Draw cube detections: bounding box + label + depth
    for (auto &det : inDet->detections) {
      int x1 = std::max(0, static_cast<int>(det.xmin * w));
      int y1 = std::max(0, static_cast<int>(det.ymin * h));
      int x2 = std::min(w, static_cast<int>(det.xmax * w));
      int y2 = std::min(h, static_cast<int>(det.ymax * h));

      const char *name =
          (det.label < class_names.size()) ? class_names[det.label] : "unknown";
      float x = det.spatialCoordinates.x * scale;
      float y = det.spatialCoordinates.y * scale;
      float z = det.spatialCoordinates.z * scale;

      cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(0, 255, 0), 2);
      std::string label =
          std::string(name) + " " +
          std::to_string(static_cast<int>(det.confidence * 100)) +
          " % X =" + std::to_string(x).substr(0, 4) + "m" +
          " % Y =" + std::to_string(y).substr(0, 4) + "m" +
          " % Z =" + std::to_string(z).substr(0, 4) + "m";
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
        std::string label = "ID=" + std::to_string(ids[i]) +
                            " Z=" + std::to_string(z_m).substr(0, 4) + "m";
        cv::putText(frame, label, cv::Point(u, v - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
      }
    }

    cv::namedWindow("debug window", cv::WINDOW_NORMAL);
    cv::imshow("debug window", frame);

    // FOR VIEWING DEPTH MAP
    //     if (!depthMat.empty()) {
    //       double minZ = 0, maxZ = 0;
    //       cv::minMaxIdx(depthMat, &minZ, &maxZ, nullptr, nullptr, depthMat >
    //       0); double scale = (maxZ > minZ) ? 255.0 / (maxZ - minZ) : 0;
    // cv::Mat depthVis;
    //       depthMat.convertTo(depthVis, CV_8U, scale, -minZ * scale);
    //       cv::resize(depthVis, depthVis, frame.size(), 0, 0,
    //       cv::INTER_NEAREST); for (size_t i = 0; i < ids.size(); i++) {
    //         float u = (corners[i][0].x + corners[i][2].x) / 2.0f;
    //         float v = (corners[i][0].y + corners[i][2].y) / 2.0f;
    //         float z_mm = lookup_depth(depthMat, u, v);
    //         cv::circle(depthVis, cv::Point(u, v), 6, cv::Scalar(255), 2);
    //         cv::putText(depthVis, "Z=" + std::to_string(z_mm *
    //         0.001f).substr(0, 4),
    //                     cv::Point(u + 8, v - 8), cv::FONT_HERSHEY_SIMPLEX,
    //                     0.4, cv::Scalar(255), 1);
    //       }
    //       cv::imshow("depth (mm)", depthVis);
    //     }

    cv::waitKey(1);
  }

private:
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
  std::string parent_tf_ = "";
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> qPreview_;
  std::shared_ptr<dai::MessageQueue> qDet_;
  std::shared_ptr<dai::MessageQueue> qDepth_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cubes_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr pub_aruco_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CubeDetectorNode>();
  node->setup();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
