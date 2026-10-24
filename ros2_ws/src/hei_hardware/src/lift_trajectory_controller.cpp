#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include "hardware_interface/component_parser.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace hei_hardware
{
// Retain the standard trajectory execution/cancellation/tolerance machinery.
// Add application input validation before the base controller reserves a goal.
class LiftTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
  using Base = joint_trajectory_controller::JointTrajectoryController;
  double minimum_{0}, maximum_{0};

  bool in_range(double value) const
  { return std::isfinite(value) && value >= minimum_ && value <= maximum_; }

  bool valid_input(const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    if (trajectory.joint_names != std::vector<std::string>{"lift_joint"} || trajectory.points.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "Reject lift trajectory: expected lift_joint and nonempty points");
      return false;
    }
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & point = trajectory.points[i];
      if (point.positions.size() != 1 || !in_range(point.positions[0])) {
        RCLCPP_WARN(get_node()->get_logger(), "Reject lift trajectory point %zu: height must be finite and in [%.6f, %.6f] m",
          i, minimum_, maximum_);
        return false;
      }
      for (const auto * field : {&point.velocities, &point.accelerations, &point.effort}) {
        for (double value : *field) {
          if (!std::isfinite(value)) {
            RCLCPP_WARN(get_node()->get_logger(), "Reject lift trajectory point %zu: nonfinite derivative/effort", i);
            return false;
          }
        }
      }
    }
    return true;
  }

public:
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous) override
  {
    using Callback = controller_interface::CallbackReturn;
    if (Base::on_configure(previous) != Callback::SUCCESS) { return Callback::ERROR; }
    try {
      bool found = false;
      for (const auto & hardware : hardware_interface::parse_control_resources_from_urdf(get_robot_description())) {
        for (const auto & joint : hardware.joints) {
          if (joint.name != "lift_joint") { continue; }
          for (const auto & command : joint.command_interfaces) {
            if (command.name != "position") { continue; }
            minimum_ = std::stod(command.min); maximum_ = std::stod(command.max); found = true;
          }
        }
      }
      if (!found || !std::isfinite(minimum_) || !std::isfinite(maximum_) || minimum_ >= maximum_ ||
        params_.joints != std::vector<std::string>{"lift_joint"} ||
        params_.command_interfaces != std::vector<std::string>{"position"}) { throw std::runtime_error("missing/invalid lift height bounds or position interface"); }
      action_server_.reset();
      action_server_ = rclcpp_action::create_server<FollowJTrajAction>(get_node(), "~/follow_joint_trajectory",
        [this](const auto & uuid, const std::shared_ptr<const FollowJTrajAction::Goal> goal) {
          if (!valid_input(goal->trajectory)) { return rclcpp_action::GoalResponse::REJECT; }
          return goal_received_callback(uuid, goal);
        },
        [this](const auto goal) { return goal_cancelled_callback(goal); },
        [this](const auto goal) { goal_accepted_callback(goal); });
      joint_command_subscriber_.reset();
      joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> message) {
          if (valid_input(*message)) { topic_callback(message); }
        });
      return Callback::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_node()->get_logger(), "Lift trajectory configuration failed: %s", error.what());
      return Callback::ERROR;
    }
  }

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const auto result = Base::update(time, period);
    if (result != controller_interface::return_type::OK || rt_is_holding_.load() || command_interfaces_.empty()) { return result; }
    // Valid endpoints can still overshoot under spline interpolation. Abort only
    // that execution, restore hold now, and leave the hardware/controller active.
    const double command = command_interfaces_[0].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    const double measured = state_current_.positions[0];
    const bool returning_to_range = std::isfinite(command) && std::isfinite(measured) &&
      command >= std::min(minimum_, measured) && command <= std::max(maximum_, measured);
    if (!in_range(command) && !returning_to_range) {
      const auto goal = *rt_active_goal_.readFromRT();
      if (goal) {
        auto response = std::make_shared<FollowJTrajAction::Result>();
        response->error_code = FollowJTrajAction::Result::INVALID_GOAL;
        response->error_string = "Lift trajectory interpolation exceeded height bounds";
        goal->setAborted(response);
        rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
        rt_has_pending_goal_ = false;
      }
      add_new_trajectory_msg(set_hold_position());
      (void)command_interfaces_[0].set_value(state_current_.positions[0]);
      RCLCPP_WARN(get_node()->get_logger(), "Lift trajectory stopped: interpolation outside [%.6f, %.6f] m", minimum_, maximum_);
    }
    return result;
  }
};
}  // namespace hei_hardware
PLUGINLIB_EXPORT_CLASS(hei_hardware::LiftTrajectoryController, controller_interface::ControllerInterface)
