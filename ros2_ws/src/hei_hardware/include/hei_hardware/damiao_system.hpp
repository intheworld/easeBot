#pragma once

#include <chrono>
#include <limits>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hei_hardware/motorbridge_backend.h"

namespace hei_hardware
{
class DamiaoSystem : public hardware_interface::SystemInterface
{
public:
  ~DamiaoSystem() override;
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;

private:
  using Clock = std::chrono::steady_clock;
  static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
  struct Motor
  {
    std::string name, command_interface;
    uint16_t id{}, feedback_id{};
    int expected_mode{0};
    bool gripper{false}, read_only{true};
    double speed_command{0.1}, current_command{0.01}, enable_command{0}, enabled_state{0};
    double current_limit{0.05}, sequence_state{0}, torque_limit{0};
    struct Limits { double position, velocity, torque; } limits{};
    unsigned index{};
    uint64_t sequence{0};
    double direction{1}, offset{0}, gear{1}, speed{0};
    double minimum{0}, maximum{0};
    double position{NaN}, velocity{NaN}, effort{NaN}, command{NaN};
    uint8_t status{0};
    uint8_t fault_status{0};
    int mode{-1};
    Clock::time_point received{}, mode_received{};
    Clock::time_point enabled_at{};
    bool enable_confirmed{false};
    bool claimed{false}, owned{false};
    bool export_effort{false}, mode_changed{false};
  };
  bool fresh(const Motor & motor) const;
  void poll();
  void receive();
  void close() noexcept;
  static void check(int result);
  bool stop_owned() noexcept;
  hardware_interface::return_type fail(const std::string & reason);
  bool validate_switch(const std::vector<std::string> & keys) const;
  void send_command(const Motor & motor, double value);

  std::vector<Motor> motors_;
  HeiMotorbridge * backend_{nullptr};
  std::string device_;
  unsigned baud_{921600};
  bool read_only_{true}, configured_{false}, active_{false}, faulted_{false};
  std::chrono::milliseconds feedback_timeout_{250}, configure_timeout_{2000};
};
}  // namespace hei_hardware
