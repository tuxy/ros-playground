#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <array>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <vector>

class ArucoDetectorNode : public rclcpp::Node {
public:
  ArucoDetectorNode() : Node("aruco_detector") {
    sub_rgb_.subscribe(this, "/oak/rgb/image_raw", rclcpp::QoS(10));
    sub_stereo_.subscribe(this, "/oak/stereo/image_raw", rclcpp::QoS(10));

    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub_rgb_, sub_stereo_);
    sync_->registerCallback(&ArucoDetectorNode::image_callback, this);

    pub_aruco_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/aruco/detections", 10);
  }

private:
  void
  image_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                 const sensor_msgs::msg::Image::ConstSharedPtr stereo_msg) {
    auto cv_rgb =
        cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    auto cv_stereo = cv_bridge::toCvCopy(
        stereo_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    auto params = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dict, params);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(cv_rgb->image, corners, ids);

    geometry_msgs::msg::PoseArray array;
    array.header = rgb_msg->header;
    pub_aruco_->publish(array);
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> sub_rgb_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_stereo_;

  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_aruco_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
