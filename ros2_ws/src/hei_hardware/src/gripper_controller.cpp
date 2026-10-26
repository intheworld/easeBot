#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace hei_hardware
{
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;
using Standard = control_msgs::action::GripperCommand;
using StandardHandle = rclcpp_action::ServerGoalHandle<Standard>;
using Clock = std::chrono::steady_clock;

// Same 50Hz executor/update model as the serial hardware. Mutexes and Action
// publication here do not provide hard real-time guarantees.
class GripperController : public controller_interface::ControllerInterface
{
public:
  Callback on_init() override
  {
    try {
      declare_fixed<std::string>("motor_resource", "");
      for (const auto & p : std::vector<std::pair<std::string, double>>{
        {"position_min_rad", -4.5}, {"position_max_rad", 0},
        {"default_velocity_rad_s", .1}, {"max_velocity_rad_s", 1},
        {"default_current_ratio", .01}, {"max_current_ratio", .01},
        {"position_tolerance_rad", .005}, {"max_timeout_sec", 120}, {"goal_timeout_sec", 0},
        {"stall_timeout_sec", 1}, {"gap_at_zero_m", 0},
        {"gap_per_motor_rad_m", 0}, {"force_per_motor_torque_n_nm", 0},
        {"motor_torque_per_current_ratio_nm", 0}}) { declare_fixed<double>(p.first, p.second); }
      declare_fixed<bool>("calibrated", false);
      declare_fixed<bool>("position_only", false);
      declare_fixed<bool>("allow_stalling", false);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
      return Callback::ERROR;
    }
    return Callback::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  { return interfaces({"position", "velocity", "current_ratio", "enabled"}); }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  { return interfaces({"position", "velocity", "effort", "enabled", "feedback_sequence"}); }

  Callback on_configure(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto node = get_node();
    resource_ = node->get_parameter("motor_resource").as_string();
    const auto value = [&](const char * name) { return node->get_parameter(name).as_double(); };
    minimum_ = value("position_min_rad"); maximum_ = value("position_max_rad");
    speed_ = value("default_velocity_rad_s"); max_speed_ = value("max_velocity_rad_s");
    current_ = value("default_current_ratio"); max_current_ = value("max_current_ratio");
    tolerance_ = value("position_tolerance_rad"); max_timeout_ = value("max_timeout_sec");
    stall_timeout_ = value("stall_timeout_sec");
    goal_timeout_ = value("goal_timeout_sec");
    calibrated_ = node->get_parameter("calibrated").as_bool();
    position_only_ = node->get_parameter("position_only").as_bool();
    allow_stalling_ = node->get_parameter("allow_stalling").as_bool();
    gap_zero_ = value("gap_at_zero_m"); gap_slope_ = value("gap_per_motor_rad_m");
    force_scale_ = value("force_per_motor_torque_n_nm");
    torque_scale_ = value("motor_torque_per_current_ratio_nm");
    for (double v : {minimum_, maximum_, speed_, max_speed_, current_, max_current_,
      tolerance_, max_timeout_, goal_timeout_, stall_timeout_, gap_zero_, gap_slope_, force_scale_, torque_scale_}) {
      if (!std::isfinite(v)) { return Callback::ERROR; }
    }
    if (resource_.empty() || minimum_ < -4.5 || maximum_ > 0 || minimum_ >= maximum_ ||
      speed_ < .01 || speed_ > max_speed_ || max_speed_ > 10 ||
      current_ < .0001 || current_ > max_current_ || max_current_ > .05 ||
      tolerance_ < .0004 || tolerance_ > .02 || max_timeout_ < 1 || max_timeout_ > 120 ||
      stall_timeout_ <= 0 || (goal_timeout_ != 0 && (goal_timeout_ < .1 || goal_timeout_ > max_timeout_))) { return Callback::ERROR; }
    const bool position_valid = gap_slope_ != 0 && gap(minimum_) >= 0 && gap(maximum_) >= 0;
    const bool force_valid = force_scale_ > 0 && torque_scale_ > 0;
    if ((calibrated_ || position_only_) && !position_valid) {
      RCLCPP_ERROR(node->get_logger(), "Invalid gripper opening/motor position mapping");
      return Callback::ERROR;
    }
    if (calibrated_ && !force_valid && !position_only_) {
      RCLCPP_ERROR(node->get_logger(), "calibrated=true requires positive force and current/torque coefficients");
      return Callback::ERROR;
    }
    calibrated_ = calibrated_ && position_valid && force_valid;
    if (position_only_ && !calibrated_) {
      RCLCPP_WARN(node->get_logger(), "Position-only: max_effort must be 0 (configured current limit); effort feedback is NaN, no Newton force control");
    }
    action_server_ = rclcpp_action::create_server<Standard>(node, "~/gripper_cmd",
      [this](const auto &, const std::shared_ptr<const Standard::Goal> goal) {
        std::lock_guard<std::mutex> guard(mutex_);
        const auto & c = goal->command;
        if (!calibrated_ && (!position_only_ || c.max_effort != 0)) {
          RCLCPP_WARN(get_node()->get_logger(), "Reject GripperCommand: gap/force calibration required");
          return rclcpp_action::GoalResponse::REJECT;
        }
        const double ratio = c.max_effort == 0 ? current_ : c.max_effort / (force_scale_ * torque_scale_);
        const bool valid = std::isfinite(c.position) && std::isfinite(c.max_effort) &&
          c.position >= 0 && c.max_effort >= 0 && valid_goal((c.position-gap_zero_)/gap_slope_, ratio);
        return reserve(valid);
      },
      [this](const std::shared_ptr<StandardHandle> goal) {
        std::lock_guard<std::mutex> guard(mutex_);
        return goal == goal_ ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<StandardHandle> goal) {
        std::lock_guard<std::mutex> guard(mutex_);
        goal_ = goal; reserved_ = false;
        const auto & c = goal->get_goal()->command;
        begin((c.position-gap_zero_)/gap_slope_,
          c.max_effort == 0 ? current_ : c.max_effort/(force_scale_*torque_scale_));
      });
    return Callback::SUCCESS;
  }

  Callback on_activate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample()) { return Callback::ERROR; }
    target_ = std::clamp(position_, minimum_, maximum_);
    commanded_speed_ = speed_; commanded_current_ = current_;
    enabled_ = false; stopping_ = false; faulted_ = false;
    active_ = true; reserved_ = false;
    send();
    return Callback::SUCCESS;
  }
  Callback on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false; reserved_ = false; enabled_ = false;
    if (!command_interfaces_.empty()) { send(); }
    finish(false, false, false, "controller deactivated; motor stop delegated to hardware");
    return Callback::SUCCESS;
  }
  Callback on_cleanup(const rclcpp_lifecycle::State &) override
  {
    action_server_.reset();
    return Callback::SUCCESS;
  }

  Return update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) { return Return::OK; }
    if (!sample()) { return Return::ERROR; }
    const auto now = Clock::now();
    if (has_goal()) {
      if (stopping_) {
        if (enabled_state_ == 0 && sequence_ > stop_sequence_) {
          finish(false, false, canceled_, reason_ + "; gripper disabled");
          stopping_ = false;
        } else if (now - stopping_at_ > std::chrono::milliseconds(500)) {
          faulted_ = true;
          finish(false, false, false, reason_ + "; disable unconfirmed");
          return Return::ERROR;
        }
      } else if (goal_->is_canceling()) {
        stop(true, "goal canceled", now);
      } else if (std::chrono::duration<double>(now-started_).count() >= timeout_) {
        stop(false, "goal timeout", now);
      } else {
        const bool reached = std::abs(position_-target_) <= tolerance_ && std::abs(velocity_) <= .03 && enabled_state_ == 1;
        if (sequence_ != previous_sequence_) { settled_ = reached ? settled_+1 : 0; }
        previous_sequence_ = sequence_;
        if (settled_ >= 3) {
          finish(true, false, false, "target reached; holding with current limit");
        } else if (allow_stalling_ && closing_) {
          // Position-only grasp: no progress while closing is a stalled closure,
          // not a measured force or proof of object contact. Keep the configured
          // current-limited target instead of disabling and dropping the object.
          if (std::abs(position_-progress_position_) > tolerance_ || std::abs(velocity_) > .03 || enabled_state_ != 1) {
            progress_position_ = position_; last_motion_ = now;
          }
          if (std::chrono::duration<double>(now-last_motion_).count() >= stall_timeout_) {
            stalled_ = true;
            finish(false, true, false, "closing stalled; holding with configured current limit; grasp unverified");
          }
        } else if (calibrated_) {
          const double limit_force = commanded_current_ * torque_scale_ * force_scale_;
          const bool stalled = enabled_state_ == 1 && std::abs(velocity_) <= .03 &&
            std::abs(torque_) * force_scale_ >= .9 * limit_force;
          if (!stalled) { last_motion_ = now; }
          if (std::chrono::duration<double>(now-last_motion_).count() >= stall_timeout_) {
            stop(false, "stalled before reaching target", now);
            stalled_ = true;
          }
        }
      }
      feedback();
    }
    send();
    return Return::OK;
  }

private:
  template<class T> void declare_fixed(const std::string & name, const T & value)
  {
    if (!get_node()->has_parameter(name)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      get_node()->declare_parameter<T>(name, value, descriptor);
    }
  }
  controller_interface::InterfaceConfiguration interfaces(std::initializer_list<const char *> types) const
  {
    std::vector<std::string> names;
    for (const auto * type : types) { names.push_back(resource_ + "/" + type); }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
  }
  double read(const std::string & type) const
  {
    for (const auto & interface : state_interfaces_) {
      if (interface.get_interface_name() == type) { return interface.get_optional().value_or(std::numeric_limits<double>::quiet_NaN()); }
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
  bool sample()
  {
    position_ = read("position"); velocity_ = read("velocity"); torque_ = read("effort");
    enabled_state_ = read("enabled"); sequence_ = read("feedback_sequence");
    sampled_at_ = Clock::now();
    return std::isfinite(position_) && std::isfinite(velocity_) && std::isfinite(torque_) &&
      sequence_ > 0 && (enabled_state_ == 0 || enabled_state_ == 1);
  }
  void send()
  {
    for (auto & interface : command_interfaces_) {
      const auto type = interface.get_interface_name();
      const double value = type == "position" ? target_ : type == "velocity" ? commanded_speed_ :
        type == "current_ratio" ? commanded_current_ : enabled_ ? 1.0 : 0.0;
      (void)interface.set_value(value);
    }
  }
  double goal_duration(double position) const
  { return goal_timeout_ == 0 ? std::max(3.0, 2*std::abs(position-position_)/speed_+2) : goal_timeout_; }
  bool valid_goal(double position, double current) const
  {
    const double duration = goal_duration(position);
    return active_ && !faulted_ && !reserved_ && !has_goal() && !stopping_ &&
      Clock::now()-sampled_at_ < std::chrono::milliseconds(250) &&
      std::isfinite(position) && std::isfinite(current) &&
      position >= minimum_ && position <= maximum_ &&
      current >= .0001 && current <= max_current_ && duration >= .1 && duration <= max_timeout_;
  }
  rclcpp_action::GoalResponse reserve(bool valid)
  {
    if (!valid) { return rclcpp_action::GoalResponse::REJECT; }
    reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  bool has_goal() const { return static_cast<bool>(goal_); }
  void begin(double position, double current)
  {
    if (!active_ || faulted_) { finish(false, false, false, "controller became inactive"); return; }
    closing_ = position > position_ + tolerance_; progress_position_ = position_;
    target_ = position; commanded_speed_ = speed_; commanded_current_ = current;
    timeout_ = goal_duration(position);
    started_ = last_motion_ = Clock::now();
    enabled_ = true; stopping_ = false; settled_ = 0; stalled_ = false;
    previous_sequence_ = sequence_;
  }
  void stop(bool canceled, const std::string & reason, Clock::time_point now)
  {
    enabled_ = false; stopping_ = true; canceled_ = canceled;
    reason_ = reason; stopping_at_ = now; stop_sequence_ = sequence_;
    target_ = std::clamp(position_, minimum_, maximum_);
  }
  void finish(bool reached, bool stalled, bool canceled, const std::string & message)
  {
    if (goal_) {
      RCLCPP_INFO(get_node()->get_logger(), "%s", message.c_str());
      auto result = std::make_shared<Standard::Result>();
      result->position = gap(position_); result->effort = measured_force();
      result->reached_goal = reached; result->stalled = stalled || stalled_;
      if (canceled && goal_->is_canceling()) { goal_->canceled(result); }
      else if (reached || (stalled && allow_stalling_)) { goal_->succeed(result); } else { goal_->abort(result); }
      goal_.reset();
    }
  }
  void feedback()
  {
    if (goal_) {
      auto f = std::make_shared<Standard::Feedback>();
      f->position = gap(position_); f->effort = measured_force();
      f->reached_goal = false; f->stalled = stalled_;
      goal_->publish_feedback(f);
    }
  }
  double measured_force() const
  { return calibrated_ ? std::abs(torque_)*force_scale_ : std::numeric_limits<double>::quiet_NaN(); }
  double gap(double motor_position) const { return gap_zero_+gap_slope_*motor_position; }
  std::mutex mutex_;
  std::string resource_, reason_;
  double minimum_{-4.5}, maximum_{0}, speed_{.1}, max_speed_{1}, current_{.01}, max_current_{.01};
  double tolerance_{.005}, max_timeout_{120}, goal_timeout_{0}, stall_timeout_{1};
  double gap_zero_{0}, gap_slope_{0}, force_scale_{0}, torque_scale_{0};
  double position_{0}, velocity_{0}, torque_{0}, enabled_state_{0}, sequence_{0};
  double previous_sequence_{0}, stop_sequence_{0}, target_{0}, commanded_speed_{.1}, commanded_current_{.01}, timeout_{0};
  bool calibrated_{false}, position_only_{false}, active_{false}, reserved_{false}, faulted_{false};
  bool allow_stalling_{false}, closing_{false};
  double progress_position_{0};
  bool enabled_{false}, stopping_{false}, canceled_{false}, stalled_{false};
  int settled_{0};
  Clock::time_point started_{}, stopping_at_{}, sampled_at_{}, last_motion_{};
  rclcpp_action::Server<Standard>::SharedPtr action_server_;
  std::shared_ptr<StandardHandle> goal_;
};
}  // namespace hei_hardware
PLUGINLIB_EXPORT_CLASS(hei_hardware::GripperController, controller_interface::ControllerInterface)
