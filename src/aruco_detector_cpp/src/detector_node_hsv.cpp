#include "aruco_msgs/msg/marker_array.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

// # Notes
// Host-side cube detector: HSV threshold per colour class, pick the largest
// contour, back-project its centroid through the aligned stereo depth to get
// a 3D pose. Runs alongside the existing ArUco detection and depth preview.
//
// HSV tuning will likely need adjustment for the actual lighting and cube
// paint. Defaults assume saturated primaries under reasonable indoor light.

struct ColorSpec {
  const char *name;
  std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;
};

struct HsvCube {
  std::string color;
  cv::Point2f centroid;
  cv::Rect bbox;
  double area = 0;
  geometry_msgs::msg::Point position;
  bool has_depth = false;
};

class HsvDetectorNode : public rclcpp::Node {
public:
  HsvDetectorNode() : Node("cube_aruco_detector_hsv") {
    declare_parameter("parent_tf", "world");
    declare_parameter("debug", false);
    declare_parameter("min_area", 500.0);

    pub_cubes_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/oakd/detections", 10);
    pub_aruco_ = this->create_publisher<aruco_msgs::msg::MarkerArray>(
        "/oakd/markers", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    debug_ = get_parameter("debug").as_bool();
    parent_tf_ = get_parameter("parent_tf").as_string();
    min_area_ = get_parameter("min_area").as_double();
  }

  void setup() {
    device_ = std::make_shared<dai::Device>();
    RCLCPP_INFO(get_logger(), "Device: %s", device_->getDeviceName().c_str());

    pipeline_ = std::make_unique<dai::Pipeline>(device_);

    auto rgbCam = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline_->create<dai::node::StereoDepth>();
    stereo->build(*monoLeft->requestOutput({640, 400}),
                  *monoRight->requestOutput({640, 400}));
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    auto rgbOut = rgbCam->requestOutput({640, 400});
    qPreview_ = rgbOut->createOutputQueue(4, false);
    qDepth_ = stereo->depth.createOutputQueue(4, false);

    auto calib = device_->readCalibration();
    auto intrinsics =
        calib.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, 640, 400);
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

    const std::vector<ColorSpec> colors = {
        {"red",
         {{cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)},
          {cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255)}}},
        {"green", {{cv::Scalar(35, 80, 70), cv::Scalar(85, 255, 255)}}},
        {"blue", {{cv::Scalar(100, 120, 70), cv::Scalar(130, 255, 255)}}},
        {"yellow", {{cv::Scalar(20, 100, 100), cv::Scalar(35, 255, 255)}}},
        {"gray", {{cv::Scalar(0, 0, 50), cv::Scalar(180, 50, 200)}}},
    };

    RCLCPP_INFO(get_logger(), "Spin loop starting");

    while (rclcpp::ok()) {
      auto inFrame = qPreview_->get<dai::ImgFrame>();
      if (!inFrame)
        continue;

      cv::Mat frame = inFrame->getCvFrame();
      cv::Mat hsv;
      cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

      auto inDepth = qDepth_->get<dai::ImgFrame>();
      cv::Mat depthMat;
      if (inDepth) {
        depthMat = inDepth->getFrame();
      }

      auto stamp = now();
      const std::string frame_id = "oak_rgb_camera_optical_frame";

      // --- HSV cube detection (largest contour per colour) ---
      std::vector<HsvCube> cubes;
      for (const auto &c : colors) {
        cv::Mat mask;
        for (const auto &r : c.ranges) {
          cv::Mat m;
          cv::inRange(hsv, r.first, r.second, m);
          if (mask.empty())
            mask = m;
          else
            cv::bitwise_or(mask, m, mask);
        }

        cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        double best_area = 0;
        int best_idx = -1;
        for (int i = 0; i < static_cast<int>(contours.size()); i++) {
          double a = cv::contourArea(contours[i]);
          if (a > best_area) {
            best_area = a;
            best_idx = i;
          }
        }
        if (best_idx < 0 || best_area < min_area_)
          continue;

        const auto &cnt = contours[best_idx];
        cv::Moments M = cv::moments(cnt);
        if (M.m00 <= 0)
          continue;
        cv::Point2f ctr(static_cast<float>(M.m10 / M.m00),
                        static_cast<float>(M.m01 / M.m00));

        HsvCube cube;
        cube.color = c.name;
        cube.centroid = ctr;
        cube.bbox = cv::boundingRect(cnt);
        cube.area = best_area;

        float z_mm = lookup_depth(depthMat, ctr.x, ctr.y);
        if (z_mm > 0) {
          cube.position.x = (ctr.x - cx_) * z_mm / fx_ * 0.001f;
          cube.position.y = (ctr.y - cy_) * z_mm / fy_ * 0.001f;
          cube.position.z = z_mm * 0.001f;
          cube.has_depth = true;
        }
        cubes.push_back(cube);

        if (cube.has_depth) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = stamp;
          pose.header.frame_id = frame_id;
          pose.pose.position = cube.position;
          pose.pose.orientation.w = 1.0;
          pub_cubes_->publish(pose);
          publish_tf(parent_tf_, std::string("cube_") + c.name, stamp,
                     cube.position, pose.pose.orientation);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Cube %s (%.0f px): x=%.3f y=%.3f z=%.3f m",
                               c.name, best_area, cube.position.x,
                               cube.position.y, cube.position.z);
        }
      }

      // --- ArUco detection ---
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

      if (debug_) {
        debug_frame(frame, cubes, ids, corners, depthMat);
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
    int half = 2;
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

  void debug_frame(cv::Mat &frame, const std::vector<HsvCube> &cubes,
                   const std::vector<int> &ids,
                   const std::vector<std::vector<cv::Point2f>> &corners,
                   const cv::Mat &depthMat) {
    for (const auto &c : cubes) {
      cv::Scalar col(0, 255, 0);
      std::string name(c.color);
      if (name == "red")
        col = cv::Scalar(0, 0, 255);
      else if (name == "blue")
        col = cv::Scalar(255, 0, 0);
      else if (name == "green")
        col = cv::Scalar(0, 255, 0);
      else if (name == "yellow")
        col = cv::Scalar(0, 255, 255);
      else if (name == "gray")
        col = cv::Scalar(200, 200, 200);
cv::circle(frame, c.centroid, 4, col, -1);
      std::string label = name;
      if (c.has_depth)
        label +=
            " Z=" + std::to_string(c.position.z).substr(0, 4) + "m";
      cv::putText(frame, label, cv::Point(c.centroid.x, c.centroid.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
    }

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

    cv::imshow("debug window", frame);

    // if (!depthMat.empty()) {
    //   double minZ = 0, maxZ = 0;
    //   cv::minMaxIdx(depthMat, &minZ, &maxZ, nullptr, nullptr, depthMat > 0);
    //   double scale = (maxZ > minZ) ? 255.0 / (maxZ - minZ) : 0;
    //   cv::Mat depthVis;
    //   depthMat.convertTo(depthVis, CV_8U, scale, -minZ * scale);
    //   cv::resize(depthVis, depthVis, frame.size(), 0, 0, cv::INTER_NEAREST);
    //   for (const auto &c : cubes) {
    //     cv::circle(depthVis, c.centroid, 6, cv::Scalar(255), 2);
    //   }
    //   for (size_t i = 0; i < ids.size(); i++) {
    //     float u = (corners[i][0].x + corners[i][2].x) / 2.0f;
    //     float v = (corners[i][0].y + corners[i][2].y) / 2.0f;
    //     cv::circle(depthVis, cv::Point(u, v), 6, cv::Scalar(255), 2);
    //   }
    //   cv::imshow("depth (mm)", depthVis);
    // }

    cv::waitKey(1);
  }

private:
  bool debug_ = false;
  std::string parent_tf_;
  double min_area_ = 500.0;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> qPreview_;
  std::shared_ptr<dai::MessageQueue> qDepth_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cubes_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr pub_aruco_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HsvDetectorNode>();
  node->setup();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
