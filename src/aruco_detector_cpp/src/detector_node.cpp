#include "shared.cpp" // pulls in shared helpers + all deps

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include <opencv2/highgui.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

static const std::string WINDOW_NAME = "OAKD-LR NN Output";

class DetectorNode : public DetectorBase {
public:
  DetectorNode()
      : DetectorBase("detector_node",
                     DetectorBaseDefaults{"world",
                                          "oak_rgb_camera_optical_frame",
                                          "/oakd", false}) {
    declare_parameter("model_path", "");
    declare_parameter("conf_threshold", 0.6);
    declare_parameter("rgb_fps", 15.0);
    declare_parameter("class_names", std::vector<std::string>{
                                         "red_cube", "green_cube", "blue_cube",
                                         "yellow_cube", "gray_cube"});

    conf_threshold_ = get_parameter("conf_threshold").as_double();
    rgb_fps_ = static_cast<float>(get_parameter("rgb_fps").as_double());
    class_names_ = get_parameter("class_names").as_string_array();
    model_path_ = get_parameter("model_path").as_string();
  }

  void initialise() {
    std::string resolved = resolveModelPath();
    RCLCPP_INFO(get_logger(), "Loading model: %s", resolved.c_str());
    if (!std::filesystem::exists(resolved)) {
      throw std::runtime_error("Model not found: " + resolved);
    }

    device_ = std::make_shared<dai::Device>();
    pipeline_ = std::make_shared<dai::Pipeline>(device_);
    RCLCPP_INFO(get_logger(), "Device: %s", device_->getDeviceName().c_str());

    PipelineConfig cfg;
    cfg.rgb_fps = rgb_fps_;
    cfg.intrinsics_width = cfg.intrinsics_height = 416;
    cfg.stereo_width = 640;
    cfg.stereo_height = 400;
    pipeline_helper_.build(pipeline_, device_, cfg);

    auto nn = dai::NNArchive(resolved);
    auto detNet = pipeline_->create<dai::node::SpatialDetectionNetwork>();
    detNet->build(pipeline_helper_.rgbCam(), pipeline_helper_.stereo(), nn);

    detection_queue_ = detNet->out.createOutputQueue(4, false);
    preview_queue_ = detNet->passthrough.createOutputQueue(4, false);
    depth_queue_ = detNet->passthroughDepth.createOutputQueue(4, false);

    auto intr = pipeline_helper_.readRgbIntrinsics();
    fx_ = intr.fx;
    fy_ = intr.fy;
    cx_ = intr.cx;
    cy_ = intr.cy;

    // Only touch the GUI when debugging, so headless runs don't fail
    if (debug()) {
      cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    }
    pipeline_->start();
    RCLCPP_INFO(get_logger(), "Pipeline started");
  }

  void run() {
    RCLCPP_INFO(get_logger(), "Spin loop starting");

    while (rclcpp::ok()) {
      auto det = detection_queue_->tryGet<dai::SpatialImgDetections>();
      auto frame = preview_queue_->tryGet<dai::ImgFrame>();
      if (!det || !frame)
        continue;

      auto depth = depth_queue_->tryGet<dai::ImgFrame>();
      cv::Mat depth_mat = depth ? depth->getFrame() : cv::Mat{};
      const auto stamp = now();
      cv::Mat cv_frame = frame->getCvFrame();

      std::vector<geometry_msgs::msg::TransformStamped> tf_batch;

      for (auto &d : det->detections) {
        if (d.confidence < conf_threshold_)
          continue;

        const float scale =
            (det->unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
        geometry_msgs::msg::Point p;
        p.x = d.spatialCoordinates.x * scale;
        p.y = d.spatialCoordinates.y * scale;
        p.z = d.spatialCoordinates.z * scale;

        if (!std::isfinite(p.z) || p.z <= 0.0f)
          continue;

        pub_cubes_->publish(makePoseStamped(stamp, p));

        const std::string frame_name =
            (static_cast<size_t>(d.label) < class_names_.size())
                ? class_names_[d.label]
                : "unknown_class";
        geometry_msgs::msg::Quaternion q;
        q.w = 1.0;
        publishTf(frame_name, stamp, p, q); // one at a time is fine here

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cube %s (%.2f): x=%.3f y=%.3f z=%.3f m",
                             frame_name.c_str(), d.confidence, p.x, p.y, p.z);
      }

      // Aruco detection
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      aruco_.detect(cv_frame, ids, corners);

      std::vector<DepthSample> samples(ids.size());
      aruco_msgs::msg::MarkerArray arr;
      arr.header.stamp = stamp;
      arr.header.frame_id = frameId();

      for (size_t i = 0; i < ids.size(); ++i) {
        const auto c = ArUcoDetector::centroid(corners[i]);
        const auto s =
            aruco_.sampleDepth(depth_mat, c.x, c.y, fx_, fy_, cx_, cy_);
        samples[i] = s;
        if (!s.valid)
          continue;

        aruco_msgs::msg::Marker marker;
        marker.header = arr.header;
        marker.id = ids[i];
        marker.pose.pose.position.x = s.x_m;
        marker.pose.pose.position.y = s.y_m;
        marker.pose.pose.position.z = s.z_m;
        marker.pose.pose.orientation.w = 1.0;
        arr.markers.push_back(marker);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parentTf();
        t.child_frame_id = "aruco_marker_" + std::to_string(ids[i]);
        t.transform.translation.x = s.x_m;
        t.transform.translation.y = s.y_m;
        t.transform.translation.z = s.z_m;
        t.transform.rotation.w = 1.0;
        tf_batch.push_back(t);
      }

      if (!arr.markers.empty())
        pub_aruco_->publish(arr);
      publishTfBatch(tf_batch);

      if (debug()) {
        debugOverlay(cv_frame, *det, ids, corners, samples);
      }
    }
  }

private:
  std::string resolveModelPath() {
    if (!model_path_.empty() && std::filesystem::exists(model_path_))
      return model_path_;

    // Prefer the package share directory.
    try {
      auto pkg =
          ament_index_cpp::get_package_share_directory("aruco_detector_cpp");
      auto p = pkg + "/models/" + "last.rvc2_legacy.rvc2.tar.xz";
      return p;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "can't find model in share: %s", e.what());
    }
    return model_path_;
  }

  void debugOverlay(cv::Mat &frame, const dai::SpatialImgDetections &dets,
                    const std::vector<int> &ids,
                    const std::vector<std::vector<cv::Point2f>> &corners,
                    const std::vector<DepthSample> &samples) {
    const float scale =
        (dets.unit == dai::LengthUnit::MILLIMETER) ? 0.001f : 1.0f;
    const int w = frame.cols, h = frame.rows;

    for (const auto &d : dets.detections) {
      if (d.confidence < conf_threshold_)
        continue;

      const int x1 = std::max(0, static_cast<int>(d.xmin * w));
      const int y1 = std::max(0, static_cast<int>(d.ymin * h));
      const int x2 = std::min(w, static_cast<int>(d.xmax * w));
      const int y2 = std::min(h, static_cast<int>(d.ymax * h));

      const std::string name =
          (static_cast<size_t>(d.label) < class_names_.size())
              ? class_names_[d.label]
              : "unknown_class";
      const float x = d.spatialCoordinates.x * scale;
      const float y = d.spatialCoordinates.y * scale;
      const float z = d.spatialCoordinates.z * scale;

      cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2),
                    cv::Scalar(0, 255, 0), 2);
      std::string label =
          name + " " + std::to_string(static_cast<int>(d.confidence * 100)) +
          "% X=" + std::to_string(x) + "m Y=" + std::to_string(y) +
          "m Z=" + std::to_string(z) + "m";
      int baseline = 0;
      cv::Size sz =
          cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
      cv::rectangle(frame, cv::Point(x1, y1 - sz.height - 4),
                    cv::Point(x1 + sz.width, y1), cv::Scalar(0, 255, 0),
                    cv::FILLED);
      cv::putText(frame, label, cv::Point(x1, y1 - 2), cv::FONT_HERSHEY_SIMPLEX,
                  0.4, cv::Scalar(0, 0, 0), 1);
    }

    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
      for (size_t i = 0; i < ids.size(); ++i) {
        const auto c = ArUcoDetector::centroid(corners[i]);
        std::string lbl = "ID=" + std::to_string(ids[i]);
        if (i < samples.size() && samples[i].valid) {
          lbl += " Z=" + std::to_string(samples[i].z_m).substr(0, 4) + "m";
        }
        cv::putText(frame, lbl, cv::Point(c.x, c.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
      }
    }

    cv::imshow(WINDOW_NAME, frame);
    cv::waitKey(1);
  }

  double conf_threshold_{0.6};
  float rgb_fps_{15.0f};
  std::string model_path_;
  std::vector<std::string> class_names_;

  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> detection_queue_;
  std::shared_ptr<dai::MessageQueue> preview_queue_;
  std::shared_ptr<dai::MessageQueue> depth_queue_;

  DepthaiPipeline pipeline_helper_;
  ArUcoDetector aruco_;

  float fx_{0}, fy_{0}, cx_{0}, cy_{0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DetectorNode>();
    node->initialise();
    node->run();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("detector_node"), "error: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
