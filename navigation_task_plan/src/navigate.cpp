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
#include <ctime>

// #include "std_msgs/msg/float32.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "lifecycle_msgs/msg/transition_event.hpp"

#include "navigate.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace navigation_task_plan
{

  NavigationController::NavigationController(const std::string & node_name)
  : rclcpp::Node(node_name)
{
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>("rosa_plansys_controller_executor");

  step_timer_cb_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  // TODO: create parameter for timer rate?
  step_timer_ = this->create_wall_timer(
    1s, std::bind(&NavigationController::step, this), step_timer_cb_group_);

}

NavigationController::~NavigationController()
{
}

void NavigationController::execute_plan(){
  // Compute the plan
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    for (auto instance: problem_expert_->getInstances()){
      std::cout<<"Instance "<< instance.name.c_str() << " type " <<
        instance.type.c_str() << std::endl;
    }
    for (auto predicate: problem_expert_->getPredicates()) {
      std::cout << "Predicates: " << std::endl;
      std::cout << parser::pddl::toString(predicate)<<std::endl;
    }

    std::cout << "Could not find plan to reach goal " <<
     parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    return;
  }

  std::cout << "Selected plan: " << std::endl;
  for (auto item : plan->items){
    RCLCPP_INFO(this->get_logger(), "  Action: '%s'", item.action.c_str());
  }
  // Execute the plan
  executor_client_->start_plan_execution(plan.value());
}

void NavigationController::finish_controlling(){
  this->step_timer_->cancel();
  this->executor_client_->cancel_plan_execution();
}

void NavigationController::step(){
  if (first_iteration_){
    this->execute_plan();
    first_iteration_ = false;
    return;
  }

  if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
    if (executor_client_->getResult().value().success) {
      RCLCPP_INFO(this->get_logger(), "Plan execution finished with success!");
      this->finish_controlling();
    } else {
        RCLCPP_INFO(this->get_logger(), "Replanning!");
        this->execute_plan();
        return;
    }
  }

  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::string error_str_ = "[" + action_feedback.action + "] finished with error: " + action_feedback.message_status;
      RCLCPP_ERROR(this->get_logger(), error_str_.c_str());
      break;
    }

    std::string arguments_str_ = " ";
    for (const auto & arguments: action_feedback.arguments){
      arguments_str_ += arguments + " ";
    }
    std::string feedback_str_ = "[" + action_feedback.action + arguments_str_ +
      std::to_string(action_feedback.completion * 100.0) + "%]";
    RCLCPP_INFO(this->get_logger(), feedback_str_.c_str());
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_task_plan::NavigationController>(
    "navigate_controller");

  rclcpp::Rate rate(5);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
}
