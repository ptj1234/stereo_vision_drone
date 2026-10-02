#include <Eigen/Eigen>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <limits>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("quad_sim_example");

  auto cmd_pub = nh->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10);

  rclcpp::sleep_for(std::chrono::seconds(2));

  while (rclcpp::ok())
  {

    /*** example 1: position control ***/
    std::cout << "\033[42m"
              << "Position Control to (2,0,1) meters"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500 && rclcpp::ok(); i++)
    {
      quadrotor_msgs::msg::PositionCommand cmd;
      cmd.position.x = 2.0;
      cmd.position.y = 0.0;
      cmd.position.z = 1.0;
      cmd_pub->publish(cmd);

      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    /*** example 1: position control ***/
    std::cout << "\033[42m"
              << "Velocity Control to (-1,0,0) meters/second"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500 && rclcpp::ok(); i++)
    {
      quadrotor_msgs::msg::PositionCommand cmd;
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = -1.0;
      cmd.velocity.y = 0.0;
      cmd.velocity.z = 0.0;
      cmd_pub->publish(cmd);

      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    /*** example 1: accelleration control ***/
    std::cout << "\033[42m"
              << "Accelleration Control to (1,0,0) meters/second^2"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500 && rclcpp::ok(); i++)
    {
      quadrotor_msgs::msg::PositionCommand cmd;
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
      cmd.acceleration.x = 1.0;
      cmd.acceleration.y = 0.0;
      cmd.acceleration.z = 0.0;
      cmd_pub->publish(cmd);

      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

  }

  rclcpp::shutdown();
  return 0;
}
