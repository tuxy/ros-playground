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
// (fast) and maybe hsv filtering on the host-side, whilst the heavy cube
// detection is offloaded to the camera. If needed, camera/rgb information can
// be ommited as to increase the available bandwidth for streaming data. Either
// use the 640x640 (default) model for longer range accuracy, or the 416x416
// model (_416) for much lower latency and higher inference FPS.
//
// # Transform publisher
// Right now, aruco detections are being published as a MarkerArray, and cube
// detections as PoseStamped. In the future, for highly accurate position
// estimation in the global map, both can publish transforms to /tf, so the
// accuracy/measurement is tied to the slam/odom provided. Aruco markers have
// ids though, so not really sure how to deal with that if using transforms.

class CubeDetectorNode : public rclcpp::Node {
public:
  CubeDetectorNode() : Node("cube_aruco_detector") {
    declare_parameter("model_path", "");
    declare_parameter("conf_threshold", 0.5);
    declare_parameter("debug", false);

    pub_cubes_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/oakd/detections", 10);
    pub_aruco_ = this->create_publisher<aruco_msgs::msg::MarkerArray>(
        "/oakd/markers", 10);

    debug_ = get_parameter("debug").as_bool();
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

    auto nnArchive = dai::NNArchive(model_path);

    // SpatialDetectionNetwork handles depth calculations
    auto detNet = pipeline_->create<dai::node::SpatialDetectionNetwork>();
    detNet->build(rgbCam, stereo, nnArchive);

    qDet_ = detNet->out.createOutputQueue(4, false);
    qPreview_ = detNet->passthrough.createOutputQueue(4, false);

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

      // NN cube detections with depth
      auto stamp = now();
      const std::string frame_id = "oak_rgb_camera_optical_frame";

      for (auto &det : inDet->detections) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;

        // spatialCoordinates are in millimeters by default
        // Convert to meters for ROS convention (possibly publish tf later on)
        float scale =
            (inDet->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
        pose.pose.position.x = det.spatialCoordinates.x * scale;
        pose.pose.position.y = det.spatialCoordinates.y * scale;
        pose.pose.position.z = det.spatialCoordinates.z * scale;

        // Leave quaternion to default / none
        pose.pose.orientation.w = 1.0;

        if (pose.pose.position.z != 0.0) {
          pub_cubes_->publish(pose);
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cube detected (%.2f): x=%.3f y=%.3f z=%.3f m",
                             det.confidence, pose.pose.position.x,
                             pose.pose.position.y, pose.pose.position.z);
      }

      // Running host-based aruco detection
      cv::Mat frame = inFrame->getCvFrame();

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(frame, dict, corners, ids, params);

      if (!ids.empty()) {
        aruco_msgs::msg::MarkerArray aruco_array;
        aruco_array.header.stamp = stamp;
        aruco_array.header.frame_id = frame_id;

        for (size_t i = 0; i < ids.size(); i++) {
          aruco_msgs::msg::Marker marker;
          marker.header = aruco_array.header;
          marker.id = ids[i];
          marker.pose.pose.position.x =
              (corners[i][0].x + corners[i][2].x) / 2.0;
          marker.pose.pose.position.y =
              (corners[i][0].y + corners[i][2].y) / 2.0;
          aruco_array.markers.push_back(marker);
        }
        pub_aruco_->publish(aruco_array);
      }

      // OpenCV Window for visualisation
      if (debug_) {
        debug_frame(frame, inDet, ids, corners);
      }

      rclcpp::spin_some(shared_from_this());
    }
  }

  void debug_frame(cv::Mat &frame,
                   const std::shared_ptr<dai::SpatialImgDetections> &inDet,
                   const std::vector<int> &ids,
                   const std::vector<std::vector<cv::Point2f>> &corners) {
    // NOTE that this section here is also useful later on if a hybrid approach
    // is preferred:
    //  - The NN cube detection runs on device
    //  - Color + confidence confirmation runs on host
    const std::array<const char *, 5> class_names = {
        "red_cube", "green_cube", "blue_cube", "yellow_cube", "gray_cube"};

    float scale = (inDet->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
    int w = frame.cols;
    int h = frame.rows;

    // Draw detections bounding box + label + depth
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

    // Draw Aruco markers
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }

    cv::imshow("debug window", frame);
    cv::waitKey(1);
  }

private:
  std::string resolve_model_path() {
    auto param_path = get_parameter("model_path").as_string();
    // Param as the priority
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
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> qPreview_;
  std::shared_ptr<dai::MessageQueue> qDet_;
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
