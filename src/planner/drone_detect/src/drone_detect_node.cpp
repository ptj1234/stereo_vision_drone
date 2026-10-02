#include <rclcpp/rclcpp.hpp>
#include "drone_detector/drone_detector.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("drone_detect");

  detect::DroneDetector drone_detector(nh);
  drone_detector.test();

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
