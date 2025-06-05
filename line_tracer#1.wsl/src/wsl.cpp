#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using namespace cv;

class WSLNode : public rclcpp::Node {
public:
  WSLNode()
  : Node("wsl_node")
  {
    error_pub_ = this->create_publisher<std_msgs::msg::Int32>("/line_error", 10);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/line_image", 10,
      std::bind(&WSLNode::imageCallback, this, _1));

    image_full_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/line_image_full", 10,
      std::bind(&WSLNode::fullImageCallback, this, _1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      Mat bin = cv_bridge::toCvShare(msg, "mono8")->image;
      Mat color_img;
      cvtColor(bin, color_img, COLOR_GRAY2BGR);

      // === 객체 탐지 및 무게중심 계산 ===
      Mat label, stats, centroids;
      Point mainPoint(bin.cols / 2, bin.rows - 1);
      int num = connectedComponentsWithStats(bin, label, stats, centroids);

      int closest = -1;
      int mindist = bin.cols;

      for (int i = 1; i < num; i++) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        if (area > 120) {
          Point center(cvRound(centroids.at<double>(i, 0)), cvRound(centroids.at<double>(i, 1)));
          int dist = norm(center - mainPoint);
          if (dist <= 140 && dist < mindist) {
            mindist = dist;
            closest = i;
          }
        }
      }

      if (closest > 0) {
        mainPoint.x = cvRound(centroids.at<double>(closest, 0));

        // === 시각화 ===
        int x = stats.at<int>(closest, CC_STAT_LEFT);
        int y = stats.at<int>(closest, CC_STAT_TOP);
        int w = stats.at<int>(closest, CC_STAT_WIDTH);
        int h = stats.at<int>(closest, CC_STAT_HEIGHT);
        rectangle(color_img, Rect(x, y, w, h), Scalar(255, 0 , 0), 2);
        circle(color_img, Point(mainPoint.x, mainPoint.y - 30), 5, Scalar(0, 0, 255), -1);
      }

      int error = (bin.cols / 2) - mainPoint.x;
      auto error_msg = std_msgs::msg::Int32();
      error_msg.data = error;
      error_pub_->publish(error_msg);

      imshow("ROI with Box and Centroid", color_img);
      waitKey(1);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void fullImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      Mat full = cv_bridge::toCvShare(msg, "bgr8")->image;
      imshow("Original Full Frame (Scaled)", full);
      waitKey(1);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge full image exception: %s", e.what());
    }
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_full_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WSLNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  destroyAllWindows();
  return 0;
}
