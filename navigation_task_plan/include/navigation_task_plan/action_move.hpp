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
#ifndef NAVIGATION_GRAPH__MOVE_HPP_
#define NAVIGATION_GRAPH__MOVE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rosa_task_plan_plansys/rosa_action.hpp"

namespace navigation_task_plan
{

using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

class Move : public rosa_task_plan_plansys::RosaAction
{
public:
  Move(const std::string & node_name,
    const std::chrono::nanoseconds & rate);

  virtual ~Move();

private:
  geometry_msgs::msg::Pose current_pos_;

  rclcpp::CallbackGroup::SharedPtr callback_group_action_client_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_cli_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;

  double dist_to_move_;

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2);
  geometry_msgs::msg::PoseStamped get_waypoint(std::string waypoint);

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void do_work() {};
};

} // end NAVIGATION_GRAPH

#endif
