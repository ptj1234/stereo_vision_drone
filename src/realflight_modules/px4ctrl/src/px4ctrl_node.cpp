#include <rclcpp/rclcpp.hpp>
#include "PX4CtrlFSM.h"

using std::placeholders::_1;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("px4ctrl");
    px4ctrl_time::clock() = nh->get_clock();

    rclcpp::sleep_for(std::chrono::seconds(1));

    Parameter_t param;
    param.config_from_ros_handle(nh);

    // Controller controller(param);
    LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);

    // QoS: mavros publishes imu and battery best-effort (SensorDataQoS), so those subscriptions
    // must be best-effort or they silently receive nothing. state/extended_state are reliable +
    // transient_local and rc/in is reliable, which a reliable subscription matches.
    // odom is best-effort so it matches any estimator publisher.
    auto state_sub =
        nh->create_subscription<mavros_msgs::msg::State>("/mavros/state",
                                                         10,
                                                         std::bind(&State_Data_t::feed, &fsm.state_data, _1));

    auto extended_state_sub =
        nh->create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",
                                                                 10,
                                                                 std::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    auto odom_sub =
        nh->create_subscription<nav_msgs::msg::Odometry>("~/odom",
                                                         rclcpp::SensorDataQoS().keep_last(100),
                                                         std::bind(&Odom_Data_t::feed, &fsm.odom_data, _1));

    auto cmd_sub =
        nh->create_subscription<quadrotor_msgs::msg::PositionCommand>("~/cmd",
                                                                      100,
                                                                      std::bind(&Command_Data_t::feed, &fsm.cmd_data, _1));

    auto imu_sub =
        nh->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                                       rclcpp::SensorDataQoS().keep_last(100),
                                                       std::bind(&Imu_Data_t::feed, &fsm.imu_data, _1));

    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh->create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",
                                                                 10,
                                                                 std::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    auto bat_sub =
        nh->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery",
                                                                rclcpp::SensorDataQoS().keep_last(100),
                                                                std::bind(&Battery_Data_t::feed, &fsm.bat_data, _1));

    auto takeoff_land_sub =
        nh->create_subscription<quadrotor_msgs::msg::TakeoffLand>("~/takeoff_land",
                                                                  100,
                                                                  std::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1));

    fsm.ctrl_FCU_pub = nh->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", 10);

    fsm.debug_pub = nh->create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    // Service clients live in a callback group that the main executor does not spin.
    // fsm.srv_executor only serves this group, so a blocking service call waits for its
    // response without running subscription callbacks in the middle of fsm.process().
    auto srv_group = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    fsm.set_FCU_mode_srv = nh->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", rmw_qos_profile_services_default, srv_group);
    fsm.arming_client_srv = nh->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", rmw_qos_profile_services_default, srv_group);
    fsm.reboot_FCU_srv = nh->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command", rmw_qos_profile_services_default, srv_group);
    fsm.srv_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    fsm.srv_executor->add_callback_group(srv_group, nh->get_node_base_interface());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    // ROS 1 ros::spinOnce() runs every queued callback. ROS 2 spin_some() takes only one
    // message per subscription, so with odom/imu faster than ctrl_freq_max the queues grow
    // and the controller runs on stale data (measured ~1 s lag). spin_all() drains them.
    auto spin_once = [&executor]()
    { executor.spin_all(std::chrono::milliseconds(5)); };
    fsm.spin_once = spin_once;

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    if (param.takeoff_land.no_RC)
    {
        RCLCPP_WARN(nh->get_logger(), "PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        RCLCPP_INFO(nh->get_logger(), "PX4CTRL] Waiting for RC");
        while (rclcpp::ok())
        {
            spin_once();
            if (fsm.rc_is_received(px4ctrl_time::now()))
            {
                RCLCPP_INFO(nh->get_logger(), "[PX4CTRL] RC received.");
                break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    int trials = 0;
    while (rclcpp::ok() && !fsm.state_data.current_state.connected)
    {
        spin_once();
        rclcpp::sleep_for(std::chrono::seconds(1));
        if (trials++ > 5)
            RCLCPP_ERROR(nh->get_logger(), "Unable to connnect to PX4!!!");
    }

    rclcpp::WallRate r(param.ctrl_freq_max);
    while (rclcpp::ok())
    {
        r.sleep();
        spin_once();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    RCLCPP_INFO(rclcpp::get_logger("px4ctrl"), "[PX4Ctrl] exit...");
    rclcpp::shutdown();
    return 0;
}
