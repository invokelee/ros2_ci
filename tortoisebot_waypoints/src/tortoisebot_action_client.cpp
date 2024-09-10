#include "tortoisebot_waypoints/tortoisebot_action_client.hpp"

void TortoisebotActionClient::send_goal() {
  using namespace std::placeholders;

  //   this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  auto goal_msg = WaypointActionIf::Goal();
  goal_msg.position.x = target_pos.x;
  goal_msg.position.y = target_pos.y;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options =
      rclcpp_action::Client<WaypointActionIf>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      std::bind(&TortoisebotActionClient::goal_response_callback, this, _1);

  send_goal_options.feedback_callback =
      std::bind(&TortoisebotActionClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
      std::bind(&TortoisebotActionClient::result_callback, this, _1);

  auto goal_handle_future =
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

bool TortoisebotActionClient::set_goal(double x, double y) {
  this->target_pos.x = x;
  this->target_pos.y = y;
  return true;
}

void TortoisebotActionClient::goal_response_callback(
    const GoalHandleWaypointAction::SharedPtr &goal_handle) {

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void TortoisebotActionClient::feedback_callback(
    GoalHandleWaypointAction::SharedPtr,
    const std::shared_ptr<const WaypointActionIf::Feedback> feedback) {

  RCLCPP_INFO(this->get_logger(),
              "Feedback received: x:%.2f, y:%.2f, theta:%.2f ",
              feedback->position.x, feedback->position.y, feedback->position.z);
}

void TortoisebotActionClient::result_callback(
    const GoalHandleWaypointAction::WrappedResult &result) {

  this->goal_done_ = true;
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    return;
  }
  std::string msg = result.result->success ? "True" : "False";

  RCLCPP_INFO(this->get_logger(),
              "Result received- status: %s, Goal Position: x:%.2f y:%.2f",
              msg.c_str(), target_pos.x, target_pos.y);
}

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto action_client = std::make_shared<TortoisebotActionClient>();

//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(action_client);

//   while (!action_client->is_goal_done()) {
//     executor.spin();
//   }

//   rclcpp::shutdown();
//   return 0;
// }