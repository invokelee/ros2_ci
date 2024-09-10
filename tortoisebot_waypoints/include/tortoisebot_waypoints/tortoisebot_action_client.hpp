#ifndef TORTOISEBOT_ACTION_CLIENT_HPP_
#define TORTOISEBOT_ACTION_CLIENT_HPP_
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "waypoint_interfaces/action/waypoint_action.hpp"

#include "nav_msgs/msg/odometry.hpp"

class TortoisebotActionClient : public rclcpp::Node {
public:
  using WaypointActionIf = waypoint_interfaces::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ClientGoalHandle<WaypointActionIf>;

  explicit TortoisebotActionClient(const rclcpp::NodeOptions &node_options);
  bool is_goal_done() const;
  void send_goal();
  bool set_goal(double x, double y);

private:
  rclcpp_action::Client<WaypointActionIf>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  geometry_msgs::msg::Point target_pos;

  void goal_response_callback(
      const GoalHandleWaypointAction::SharedPtr &goal_handle);

  void feedback_callback(
      GoalHandleWaypointAction::SharedPtr,
      const std::shared_ptr<const WaypointActionIf::Feedback> feedback);

  void result_callback(const GoalHandleWaypointAction::WrappedResult &result);

}; // class TortoisebotActionClient

inline TortoisebotActionClient::TortoisebotActionClient(
    const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
    : Node("actions_quiz_client", node_options), goal_done_(false) {

  target_pos.x = 0.5;
  target_pos.y = 0.5;
  this->client_ptr_ = rclcpp_action::create_client<WaypointActionIf>(
      this->get_node_base_interface(), this->get_node_graph_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(),
      "/tortoisebot_as");

  //   this->timer_ = this->create_wall_timer(
  //       std::chrono::milliseconds(500),
  //       std::bind(&TortoisebotActionClient::send_goal, this));
}

inline bool TortoisebotActionClient::is_goal_done() const {
  return this->goal_done_;
}

#endif // TORTOISEBOT_ACTION_CLIENT_HPP_