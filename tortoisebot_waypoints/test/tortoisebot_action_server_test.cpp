#include "gtest/gtest.h"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tortoisebot_waypoints/tortoisebot_action_client.hpp"
#include "tortoisebot_waypoints/tortoisebot_action_server.hpp"
#include "waypoint_interfaces/action/detail/waypoint_action__struct.hpp"
#include "waypoint_interfaces/action/waypoint_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class TortoisebotActionClientFixture : public ::testing::Test {
public:
  using WaypointActionIf = waypoint_interfaces::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ClientGoalHandle<WaypointActionIf>;

  TortoisebotActionClientFixture() {
    goal_done_ = false;
    rclcpp::Rate loop_rate(10);
    int cnt = 0;

    action_server_node = std::make_shared<WaypointAction>();
    action_client_node = std::make_shared<TortoisebotActionClient>();
    odom_sub_node = rclcpp::Node::make_shared("test_odom_subs");
    ;

    while (cnt++ < 50) {
      rclcpp::spin_some(action_server_node);
      rclcpp::spin_some(action_client_node);
      loop_rate.sleep();
    }
    odom_subscription_ =
        odom_sub_node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TortoisebotActionClientFixture::odom_topic_callback,
                      this, _1));
  }

  void odom_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double CorrectPositionTest(float tx, float ty);
  double CorrectRotationTest(float deg);

private:
  std::shared_ptr<WaypointAction> action_server_node;
  std::shared_ptr<TortoisebotActionClient> action_client_node;
  std::shared_ptr<rclcpp::Node> odom_sub_node;
  bool goal_done_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  geometry_msgs::msg::Point current_pos_;
  double current_yaw_, err_yaw_;

  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
    rclcpp::Rate loop_rate(10);
    int cnt = 0;

    while (cnt++ < 20) {
      rclcpp::spin_some(action_server_node);
      rclcpp::spin_some(action_client_node);
      rclcpp::spin_some(odom_sub_node);
      loop_rate.sleep();
    }
    action_client_node->set_goal(0.5, 0.5);
    action_client_node->send_goal();

    while (!action_client_node->is_goal_done()) {
      rclcpp::spin_some(action_server_node);
      rclcpp::spin_some(action_client_node);
      rclcpp::spin_some(odom_sub_node);
      loop_rate.sleep();
    }
  }
}; // class TortoisebotActionClientFixture

void TortoisebotActionClientFixture::odom_topic_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pos_ = msg->pose.pose.position;

  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw_ = yaw;
}

double TortoisebotActionClientFixture::CorrectPositionTest(float tx, float ty) {
  rclcpp::Rate loop_rate(10);
  int cnt = 0;

  cnt = 0;
  while (cnt++ < 20) {
    rclcpp::spin_some(action_server_node);
    rclcpp::spin_some(action_client_node);
    rclcpp::spin_some(odom_sub_node);
    loop_rate.sleep();
  }
  auto d_y = atan2(ty - current_pos_.y, tx - current_pos_.x);
  auto e_p = sqrt(pow(ty - current_pos_.y, 2) + pow(tx - current_pos_.x, 2));
  err_yaw_ = d_y - current_yaw_;

  return e_p;
}

double TortoisebotActionClientFixture::CorrectRotationTest(float deg) {
  rclcpp::Rate loop_rate(10);
  int cnt = 0;
  (void)deg;

  while (cnt++ < 20) {
    rclcpp::spin_some(action_server_node);
    rclcpp::spin_some(action_client_node);
    rclcpp::spin_some(odom_sub_node);
    loop_rate.sleep();
  }

  return err_yaw_;
}

TEST_F(TortoisebotActionClientFixture, EndPositionTest) {
  EXPECT_GE(0.05, CorrectPositionTest(0.5, 0.5)); // +- 5 cm
}

TEST_F(TortoisebotActionClientFixture, EndRotationTest) {
  EXPECT_GE(0.1, CorrectRotationTest(0.1)); // 0.1 : +- 5.7 degree
}
