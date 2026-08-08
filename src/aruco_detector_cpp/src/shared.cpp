#include "aruco_msgs/msg/marker_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace {
struct DetectorBaseDefaults {
  std::string parent_tf = "world";
  std::string frame_id = "oak_rgb_camera_optical_frame";
  std::string topic_namespace = "/oakd";
  bool debug = false;
};

class DetectorBase : public rclcpp::Node {
public:
  explicit DetectorBase(const std::string &node_name,
                        const DetectorBaseDefaults &defaults = {})
      : rclcpp::Node(node_name) {
    declare_parameter("parent_tf", defaults.parent_tf);
    declare_parameter("frame_id", defaults.frame_id);
    declare_parameter("topic_namespace", defaults.topic_namespace);
    declare_parameter("debug", defaults.debug);

    parent_tf_ = get_parameter("parent_tf").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    topic_namespace_ = get_parameter("topic_namespace").as_string();
    debug_ = get_parameter("debug").as_bool();

    const auto qos = rclcpp::QoS(10).best_effort().durability_volatile();
    pub_cubes_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        defaults.topic_namespace + "/detections", qos);
    pub_aruco_ = this->create_publisher<aruco_msgs::msg::MarkerArray>(
        defaults.topic_namespace + "/markers", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void publishTf(const std::string &child,
                 const builtin_interfaces::msg::Time &stamp,
                 const geometry_msgs::msg::Point &pos,
                 const geometry_msgs::msg::Quaternion &orient) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent_tf_;
    t.child_frame_id = child;
    t.transform.translation.x = pos.x;
    t.transform.translation.y = pos.y;
    t.transform.translation.z = pos.z;
    t.transform.rotation = orient;
    tf_broadcaster_->sendTransform(t);
  }

  void publishTfBatch(
      const std::vector<geometry_msgs::msg::TransformStamped> &tf_list) {
    if (!tf_list.empty())
      tf_broadcaster_->sendTransform(tf_list);
  }

  geometry_msgs::msg::PoseStamped
  makePoseStamped(const builtin_interfaces::msg::Time &stamp,
                  const geometry_msgs::msg::Point &pos) const {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = stamp;
    p.header.frame_id = frame_id_;
    p.pose.position = pos;
    p.pose.orientation.w = 1.0;
    return p;
  }

  const std::string &parentTf() const { return parent_tf_; }
  const std::string &frameId() const { return frame_id_; }
  bool debug() const { return debug_; }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cubes_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr pub_aruco_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string parent_tf_;
  std::string frame_id_;
  std::string topic_namespace_;
  bool debug_{false};
};

struct PipelineConfig {
  // fx/fy/cx/cy intrinsics
  int intrinsics_width = 416;
  int intrinsics_height = 416;
  // depth output size.
  int stereo_width = 640;
  int stereo_height = 400;
  // FPS throttle
  float rgb_fps = 0.0f;
};

struct CameraIntrinsics {
  float fx{0}, fy{0}, cx{0}, cy{0};
};

class DepthaiPipeline {
public:
  void build(std::shared_ptr<dai::Pipeline> pipeline,
             std::shared_ptr<dai::Device> device, const PipelineConfig &cfg) {
    cfg_ = cfg;
    device_ = device;

    rgb_cam_ = pipeline->create<dai::node::Camera>();
    rgb_cam_->build(dai::CameraBoardSocket::CAM_A, std::nullopt, cfg.rgb_fps);

    mono_left_ = pipeline->create<dai::node::Camera>();
    mono_right_ = pipeline->create<dai::node::Camera>();
    mono_left_->build(dai::CameraBoardSocket::CAM_B);
    mono_right_->build(dai::CameraBoardSocket::CAM_C);

    stereo_ = pipeline->create<dai::node::StereoDepth>();
    stereo_->build(
        *mono_left_->requestOutput({cfg.stereo_width, cfg.stereo_height}),
        *mono_right_->requestOutput({cfg.stereo_width, cfg.stereo_height}));
    stereo_->setDepthAlign(dai::CameraBoardSocket::CAM_A);
  }

  std::shared_ptr<dai::node::Camera> rgbCam() const { return rgb_cam_; }
  std::shared_ptr<dai::node::StereoDepth> stereo() const { return stereo_; }

  CameraIntrinsics readRgbIntrinsics() {
    auto calib = device_->readCalibration();
    auto intr = calib.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A,
                                          cfg_.intrinsics_width,
                                          cfg_.intrinsics_height);
    CameraIntrinsics out{intr[0][0], intr[1][1], intr[0][2], intr[1][2]};
    RCLCPP_INFO(rclcpp::get_logger("depthai_pipeline"),
                "Intrinsics @ %dx%d: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                cfg_.intrinsics_width, cfg_.intrinsics_height, out.fx, out.fy,
                out.cx, out.cy);
    return out;
  }

private:
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::node::Camera> rgb_cam_;
  std::shared_ptr<dai::node::Camera> mono_left_;
  std::shared_ptr<dai::node::Camera> mono_right_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  PipelineConfig cfg_{};
};

struct DepthSample {
  bool valid{false};
  float z_mm{0};
  float x_m{0};
  float y_m{0};
  float z_m{0};
};

class ArUcoDetector {
public:
  ArUcoDetector()
      : dict_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
        params_(cv::aruco::DetectorParameters::create()) {}

  void detect(const cv::Mat &frame, std::vector<int> &ids,
              std::vector<std::vector<cv::Point2f>> &corners) {
    cv::aruco::detectMarkers(frame, dict_, corners, ids, params_);
  }

  static cv::Point2f centroid(const std::vector<cv::Point2f> &c) {
    return {(c[0].x + c[2].x) * 0.5f, (c[0].y + c[2].y) * 0.5f};
  }

  // Works well as a throttle for an unreliable model, but will need to be
  // simplified when a better model is in place
  DepthSample sampleDepth(const cv::Mat &depth_mm, float u, float v, float fx,
                          float fy, float cx, float cy) {
    if (depth_mm.empty() || depth_mm.type() != CV_16UC1)
      return {};

    const int px = static_cast<int>(u);
    const int py = static_cast<int>(v);
    constexpr int half = 2;
    constexpr int kSamples = (2 * half + 1) * (2 * half + 1); // 25

    std::array<uint16_t, kSamples> samples{};
    int n = 0;
    for (int dy = -half; dy <= half; ++dy) {
      const int y = py + dy;
      if (y < 0 || y >= depth_mm.rows)
        continue;
      const uint16_t *row = depth_mm.ptr<uint16_t>(y);
      for (int dx = -half; dx <= half; ++dx) {
        const int x = px + dx;
        if (x < 0 || x >= depth_mm.cols)
          continue;
        const uint16_t z = row[x];
        if (z > 0)
          samples[n++] = z;
      }
    }
    if (n < (kSamples / 2 + 1))
      return {};

    std::nth_element(samples.begin(), samples.begin() + n / 2,
                     samples.begin() + n);
    const uint16_t z_mm = samples[n / 2];

    DepthSample out;
    out.valid = true;
    out.z_mm = static_cast<float>(z_mm);
    out.x_m = (u - cx) * out.z_mm / fx * 0.001f;
    out.y_m = (v - cy) * out.z_mm / fy * 0.001f;
    out.z_m = out.z_mm * 0.001f;
    return out;
  }

private:
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
};

} // namespace
