// Copyright 2024 Gustavo Rezende Silva
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "navigation_task_plan/action_move.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace navigation_task_plan
{

  Move::Move(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate)
  {
  }

  Move::~Move()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Move::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    callback_group_action_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    navigate_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose",
      callback_group_action_client_
    );

    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
     "/amcl_pose",
     10,
     std::bind(&Move::current_pos_callback, this, _1));

    ros_typedb_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    typedb_query_cli_ = this->create_client<ros_typedb_msgs::srv::Query>(
        "ros_typedb/query",
        rmw_qos_profile_services_default,
        ros_typedb_cb_group_);

    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  void Move::current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  double Move::getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  std::optional<geometry_msgs::msg::PoseStamped> Move::fetch_pose(std::string room_name) {
    if(!typedb_query_cli_->service_is_ready()){
      return {};
    }

    auto request = std::make_shared<ros_typedb_msgs::srv::Query::Request>();
    request->query_type = "fetch";
    request->query = "match "
      "$room isa room, has room-name '" + room_name + "' ;"
      "$pose (physical_thing:$room) isa pose2d;"
      "fetch $pose:x, y, theta;";

    auto typedb_query_result_ = typedb_query_cli_->async_send_request(request);

    // Wait for the result.
    if (typedb_query_result_.wait_for(1s) != std::future_status::ready) {
      return {};
    }

    auto result_ = typedb_query_result_.get();
    if(!result_->success){
      return {};
    }

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    for (const auto& result: result_->results) {
      for (const auto& attribute: result.attributes) {
        if (attribute.label == "x") {
          wp.pose.position.x = attribute.value.double_value;
        } else if (attribute.label == "y") {
          wp.pose.position.y = attribute.value.double_value;
        } else if (attribute.label == "theta") {
          wp.pose.orientation.z = attribute.value.double_value;
        }
      }
    }
    return wp;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Move::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Move starting");

    while(!navigate_cli_->wait_for_action_server(std::chrono::seconds(5))){
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");
    }

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    nav2_msgs::action::NavigateToPose::Goal navigation_goal;
    auto pose = fetch_pose(get_arguments()[1]);
    if (!pose.has_value()) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    navigation_goal.pose = pose.value();
    dist_to_move_ = getDistance(navigation_goal.pose.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move_))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigate_cli_->async_send_goal(navigation_goal, send_goal_options);

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Move::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    navigate_cli_->async_cancel_all_goals();

   return plansys2::ActionExecutorClient::on_deactivate(previous_state);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_task_plan::Move>(
    "move", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
