#include "tortoisebot_waypoints/tortoisebot_action_server.hpp"
#include "rclcpp/node_options.hpp"

void WaypointAction::odom_topic_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pos_ = msg->pose.pose.position;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;
}

rclcpp_action::GoalResponse WaypointAction::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const WaypointActionIf::Goal> goal) {

  desired_pos_ = goal->position;
  calc_distance_to_goal();

  RCLCPP_INFO(this->get_logger(),
              "Received goal request with x: %f, y: %f, theta: %f",
              desired_pos_.x, desired_pos_.y, desired_yaw_);

  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointAction::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointAction::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&WaypointAction::execute, this, _1), goal_handle}
      .detach();
}

void WaypointAction::calc_distance_to_goal() {
  desired_yaw_ =
      atan2(desired_pos_.y - current_pos_.y, desired_pos_.x - current_pos_.x);

  err_pos_ = sqrt(pow(desired_pos_.y - current_pos_.y, 2) +
                  pow(desired_pos_.x - current_pos_.x, 2));
  err_yaw_ = desired_yaw_ - current_yaw_;
}

void WaypointAction::execute(
    const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto feedback = std::make_shared<WaypointActionIf::Feedback>();
  auto &fe = feedback;
  auto result = std::make_shared<WaypointActionIf::Result>();
  auto move = geometry_msgs::msg::Twist();

  rclcpp::Rate loop_rate(10);
  int cnt = 0;
  calc_distance_to_goal();

  while (rclcpp::ok() && err_pos_ > _dist_precision) { // +-5 cm 0.05
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    if (fabs(err_yaw_) > _yaw_precision) { // +- 5 degree 0.09
      if (cnt % 10 == 0)
        RCLCPP_INFO(this->get_logger(), "Direction correction");
      move.linear.x = 0.0;
      move.angular.z = 0.65;
      if (err_yaw_ < 0)
        move.angular.z *= -1;
      publisher_->publish(move);
    } else {
      if (cnt % 10 == 0)
        RCLCPP_INFO(this->get_logger(), "Go to Desired Position");
      move.linear.x = 0.6;
      move.angular.z = 0.0;
      if (err_yaw_ > 0)
        move.angular.z = 0.1;
      else if (err_yaw_ < 0)
        move.angular.z = -0.1;
      publisher_->publish(move);
    }
    if (cnt++ % 10 == 0) {
      fe->position.x = current_pos_.x;
      fe->position.y = current_pos_.y;
      fe->position.z = current_yaw_;
      fe->state = "go to Waypoint";
      goal_handle->publish_feedback(feedback);
    }

    loop_rate.sleep();
    calc_distance_to_goal();
    if (err_pos_ <= _dist_precision)
      break;
  }
  // Stop moving
  move.angular.z = 0.0;
  move.linear.x = 0.0;
  publisher_->publish(move);

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}
