#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "line_tracer/dxl.hpp"

using namespace std;
using namespace cv;

class JetsonNode : public rclcpp::Node {
public:
  JetsonNode()
  : Node("jetson_node"),
    received_error_(0),
    dxl_()
  {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/line_image", 10);
    image_full_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/line_image_full", 10);

    error_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/line_error", 10,
      std::bind(&JetsonNode::errorCallback, this, std::placeholders::_1));

    if (!dxl_.open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open Dynamixel.");
      rclcpp::shutdown();
    }

    cap_.open("5.mp4");
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video.");
      rclcpp::shutdown();
    }
  }

  void spin() {
    rclcpp::Rate rate(30);
    Mat frame, gray, bin;

    while (rclcpp::ok()) {
      cap_ >> frame;
      if (frame.empty()) {
        RCLCPP_INFO(this->get_logger(), "Video ended.");
        break;
      }

      // === 축소된 원본 프레임 생성 (시각화용) ===
      Mat small_frame;
      resize(frame, small_frame, Size(640, 360));
      auto full_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", small_frame).toImageMsg();
      image_full_pub_->publish(*full_msg);

      // === ROI 이진 영상 생성 ===
      cvtColor(frame, gray, COLOR_BGR2GRAY);
      Scalar avg = mean(gray);
      gray += (100 - avg[0]);
      threshold(gray, bin, 128, 255, THRESH_BINARY);
      Mat roi = bin(Rect(0, bin.rows - 90, 640, 90));
      auto roi_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", roi).toImageMsg();
      image_pub_->publish(*roi_msg);

      // === Dynamixel 제어 ===
      double k = 0.4;
      int leftvel = 100 - k * received_error_;
      int rightvel = -(100 + k * received_error_);
      RCLCPP_INFO(this->get_logger(), "Set velocity -> left: %d, right: %d", leftvel, rightvel);
      dxl_.setVelocity(leftvel, rightvel);

      rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }

    dxl_.close();
  }

private:
  void errorCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    received_error_ = msg->data;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_full_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr error_sub_;
  VideoCapture cap_;
  int received_error_;
  Dxl dxl_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JetsonNode>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
