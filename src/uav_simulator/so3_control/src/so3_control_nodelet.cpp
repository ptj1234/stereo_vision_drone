#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <so3_control/SO3Control.h>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <plan_env/param_helper.hpp>

// ROS 1 nodelet "so3_control/SO3ControlNodelet" as an rclcpp component
class SO3ControlNodelet : public rclcpp::Node
{
public:
  explicit SO3ControlNodelet(const rclcpp::NodeOptions& options)
    : rclcpp::Node("so3_control", options)
    , position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , // FIXME
    use_external_yaw_(false)
  {
    onInit();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void publishSO3Command(void);
  void onInit(void);
  void position_cmd_callback(
    const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void enable_motors_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void corrections_callback(const quadrotor_msgs::msg::Corrections::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

  SO3Control      controller_;
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr      so3_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               enable_motors_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr  corrections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr             imu_sub_;

  bool        position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double          des_yaw_, des_yaw_dot_;
  double          current_yaw_;
  bool            enable_motors_;
  bool            use_external_yaw_;
  double          kR_[3], kOm_[3], corrections_[3];
  double          init_x_, init_y_, init_z_;
};

void
SO3ControlNodelet::publishSO3Command(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  auto so3_command = std::make_unique<quadrotor_msgs::msg::SO3Command>();
  so3_command->header.stamp    = now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x         = force(0);
  so3_command->force.y         = force(1);
  so3_command->force.z         = force(2);
  so3_command->orientation.x   = orientation.x();
  so3_command->orientation.y   = orientation.y();
  so3_command->orientation.z   = orientation.z();
  so3_command->orientation.w   = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    so3_command->kr[i]  = kR_[i];
    so3_command->kom[i] = kOm_[i];
  }
  so3_command->aux.current_yaw          = current_yaw_;
  so3_command->aux.kf_correction        = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors        = enable_motors_;
  so3_command->aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_->publish(std::move(so3_command));
}

void
SO3ControlNodelet::position_cmd_callback(
  const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  if ( cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5 )
  {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if ( cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5 )
  {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishSO3Command();
}

void
SO3ControlNodelet::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf2::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_)
      publishSO3Command();
    position_cmd_updated_ = false;
  }
  else if ( init_z_ > -9999.0 )
  {
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0,0,0);
    des_acc_ = Eigen::Vector3d(0,0,0);
    publishSO3Command();
  }
  
}

void
SO3ControlNodelet::enable_motors_callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (msg->data)
    RCLCPP_INFO(get_logger(), "Enabling motors");
  else
    RCLCPP_INFO(get_logger(), "Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlNodelet::corrections_callback(
  const quadrotor_msgs::msg::Corrections::ConstSharedPtr msg)
{
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlNodelet::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  const sensor_msgs::msg::Imu& imu = *imu_msg;
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

void
SO3ControlNodelet::onInit(void)
{
  auto n = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});

  std::string quadrotor_name;
  param_helper::param(n, "quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = quadrotor_name;  // tf2 frame ids may not start with '/'

  double mass;
  param_helper::param(n, "mass", mass, 0.5);
  controller_.setMass(mass);

  param_helper::param(n, "use_external_yaw", use_external_yaw_, true);

  param_helper::param(n, "gains/rot/x", kR_[0], 1.5);
  param_helper::param(n, "gains/rot/y", kR_[1], 1.5);
  param_helper::param(n, "gains/rot/z", kR_[2], 1.0);
  param_helper::param(n, "gains/ang/x", kOm_[0], 0.13);
  param_helper::param(n, "gains/ang/y", kOm_[1], 0.13);
  param_helper::param(n, "gains/ang/z", kOm_[2], 0.1);
  param_helper::param(n, "gains/kx/x", kx_[0], 5.7);
  param_helper::param(n, "gains/kx/y", kx_[1], 5.7);
  param_helper::param(n, "gains/kx/z", kx_[2], 6.2);
  param_helper::param(n, "gains/kv/x", kv_[0], 3.4);
  param_helper::param(n, "gains/kv/y", kv_[1], 3.4);
  param_helper::param(n, "gains/kv/z", kv_[2], 4.0);

  param_helper::param(n, "corrections/z", corrections_[0], 0.0);
  param_helper::param(n, "corrections/r", corrections_[1], 0.0);
  param_helper::param(n, "corrections/p", corrections_[2], 0.0);

  param_helper::param(n, "so3_control/init_state_x", init_x_, 0.0);
  param_helper::param(n, "so3_control/init_state_y", init_y_, 0.0);
  param_helper::param(n, "so3_control/init_state_z", init_z_, -10000.0);

  so3_command_pub_ = create_publisher<quadrotor_msgs::msg::SO3Command>("~/so3_cmd", 10);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("~/odom", 10, std::bind(&SO3ControlNodelet::odom_callback, this, std::placeholders::_1));
  position_cmd_sub_ =
    create_subscription<quadrotor_msgs::msg::PositionCommand>("~/position_cmd", 10, std::bind(&SO3ControlNodelet::position_cmd_callback, this, std::placeholders::_1));

  enable_motors_sub_ =
    create_subscription<std_msgs::msg::Bool>("~/motors", 2, std::bind(&SO3ControlNodelet::enable_motors_callback, this, std::placeholders::_1));
  corrections_sub_ =
    create_subscription<quadrotor_msgs::msg::Corrections>("~/corrections", 10, std::bind(&SO3ControlNodelet::corrections_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("~/imu", 10, std::bind(&SO3ControlNodelet::imu_callback, this, std::placeholders::_1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SO3ControlNodelet)
