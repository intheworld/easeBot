#pragma once
#include "hei_hardware/damiao_system.hpp"

namespace hei_hardware
{
// Lift-only coordinate/limit adapter. The composed DamiaoSystem retains sole
// ownership of motor transport, protocol, mode checks and enable acknowledgements.
class LiftSystem : public hardware_interface::SystemInterface
{
public:
  ~LiftSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &) override;
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
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;
private:
  using Clock = std::chrono::steady_clock;
  enum class Phase { Unhomed, Homing, Homed, Fault };
  DamiaoSystem motor_;
  std::vector<hardware_interface::StateInterface> motor_states_;
  std::vector<hardware_interface::CommandInterface> motor_commands_;
  std::string io_port_;
  int io_fd_{-1}, upper_mask_{1}, lower_mask_{2};
  std::vector<unsigned char> io_buffer_;
  Clock::time_point io_received_{}, home_started_{}, last_write_{};
  Clock::time_point rejected_warn_at_{};
  Phase phase_{Phase::Unhomed};
  bool readonly_{true}, home_on_activate_{false}, active_{false}, claimed_{false}, motor_claimed_{false};
  bool have_raw_{false}, homing_stop_{false};
  double position_{0}, velocity_{0}, command_{0}, upper_{0}, lower_{0}, online_{0}, homed_{0}, phase_state_{0};
  double raw_{0}, raw_torque_{0}, last_raw_{0}, turns_{0}, zero_{0}, last_speed_{0};
  double startup_hold_{std::numeric_limits<double>::quiet_NaN()};
  double scale_{0}, up_sign_{-1}, wrap_{25}, speed_{18}, home_speed_{10}, accel_{30}, kp_{450}, tolerance_{.001};
  double minimum_{-.8}, maximum_{0}, io_timeout_{.5}, home_timeout_{30}, configure_timeout_{3};
  void open_io();
  void close_io() noexcept;
  void poll_io();
  void sample();
  void set_homed();
  bool io_fresh() const;
  hardware_interface::return_type claim_motor();
  hardware_interface::return_type fault(const std::string &);
  static bool valid_keys(const std::vector<std::string> &);
};
}  // namespace hei_hardware
