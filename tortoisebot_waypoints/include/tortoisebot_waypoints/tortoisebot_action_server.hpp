#ifndef TORTOISEBOT_ACTION_SERVER_HPP_
#define TORTOISEBOT_ACTION_SERVER_HPP_
#include <math.h>
#include <memory>
#include <thread>

#include "rcl_action/action_server.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rclcpp_action/server_goal_handle.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>

#include "waypoint_interfaces/action/detail/waypoint_action__struct.hpp"
#include "waypoint_interfaces/action/waypoint_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class WaypointAction : public rclcpp::Node {
public:
  using WaypointActionIf = waypoint_interfaces::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ServerGoalHandle<WaypointActionIf>;

  explicit WaypointAction(const rclcpp::NodeOptions &options);

private:
  rclcpp_action::Server<WaypointActionIf>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Point desired_pos_;
  double current_yaw_, desired_yaw_;
  double err_pos_, err_yaw_;
  double _dist_precision, _yaw_precision;

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointActionIf::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

  void calc_distance_to_goal();

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);

}; // class WaypointAction

inline WaypointAction::WaypointAction(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("waypoint_action_node", options) {

  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<WaypointActionIf>(
      this, "/tortoisebot_as",
      std::bind(&WaypointAction::handle_goal, this, _1, _2),
      std::bind(&WaypointAction::handle_cancel, this, _1),
      std::bind(&WaypointAction::handle_accepted, this, _1));

  _dist_precision = 0.05; // +- 5 cm
  _yaw_precision = 0.035; // +- 2 degree

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&WaypointAction::odom_topic_callback, this, _1));

  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "Tortoisebot Waypoint Action server started");
}

#endif // TORTOISEBOT_ACTION_SERVER_HPP_