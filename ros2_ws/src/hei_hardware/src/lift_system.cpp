#include "hei_hardware/lift_system.hpp"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include "pluginlib/class_list_macros.hpp"

namespace hei_hardware
{
namespace
{
using Callback = hardware_interface::CallbackReturn;
using Result = hardware_interface::return_type;
const auto logger = rclcpp::get_logger("hei_hardware.lift");
double number(const std::unordered_map<std::string, std::string> & p, const std::string & key)
{
  const auto & text = p.at(key);
  size_t end;
  const double value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) { throw std::invalid_argument("invalid " + key); }
  return value;
}
bool flag(const std::unordered_map<std::string, std::string> & p, const std::string & key)
{
  const auto & v = p.at(key);
  if (v != "true" && v != "false") { throw std::invalid_argument("invalid " + key); }
  return v == "true";
}
uint16_t crc(const unsigned char * bytes, size_t count)
{
  uint16_t value = 0xffff;
  for (size_t i = 0; i < count; ++i) {
    value ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit) { value = (value >> 1) ^ ((value & 1) ? 0xa001 : 0); }
  }
  return value;
}
}  // namespace

LiftSystem::~LiftSystem() { close_io(); }
Callback LiftSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & p)
{
  if (SystemInterface::on_init(p) != Callback::SUCCESS) { return Callback::ERROR; }
  try {
    const auto & h = info_.hardware_parameters;
    readonly_ = flag(h, "read_only");
    home_on_activate_ = flag(h, "home_on_activate") && !readonly_;
    io_port_ = h.at("io_port");
    if (number(h, "io_baud") != 115200 || io_port_.empty() || io_port_[0] != '/') { throw std::invalid_argument("IO requires an absolute port and 115200 baud"); }
    const double upper_bit = number(h, "upper_bit"), lower_bit = number(h, "lower_bit");
    if (!((upper_bit == 1 && lower_bit == 2) || (upper_bit == 2 && lower_bit == 1))) { throw std::invalid_argument("limit bits must be 1 and 2"); }
    upper_mask_ = upper_bit == 1 ? 1 : 2; lower_mask_ = lower_bit == 1 ? 1 : 2;
    up_sign_ = number(h, "up_sign"); wrap_ = number(h, "position_wrap_rad");
    const double lead = number(h, "lead_m_per_rev");
    speed_ = number(h, "max_speed_rad_s"); home_speed_ = number(h, "home_speed_rad_s");
    accel_ = number(h, "max_accel_rad_s2"); kp_ = number(h, "position_kp_rad_s_per_m");
    tolerance_ = number(h, "position_tolerance_m"); minimum_ = number(h, "minimum_m"); maximum_ = number(h, "maximum_m");
    io_timeout_ = number(h, "io_timeout_sec"); home_timeout_ = number(h, "home_timeout_sec");
    configure_timeout_ = number(h, "io_wait_timeout_sec");
    if (std::abs(up_sign_) != 1 || wrap_ != 25 || lead <= 0 || speed_ <= 0 || speed_ > 30 ||
      home_speed_ <= 0 || home_speed_ > speed_ || accel_ <= 0 || kp_ <= 0 || tolerance_ <= 0 ||
      minimum_ >= 0 || maximum_ != 0 || tolerance_ >= -minimum_ || io_timeout_ <= 0 ||
      home_timeout_ <= 0 || configure_timeout_ <= 0) { throw std::invalid_argument("invalid lift geometry/control settings"); }
    if (info_.joints.size() != 1 || info_.joints[0].name != "lift_joint" ||
      info_.gpios.size() != 1 || info_.gpios[0].name != "lift_status") { throw std::invalid_argument("expected lift_joint and lift_status"); }
    const auto & commands = info_.joints[0].command_interfaces;
    if ((readonly_ && !commands.empty()) || (!readonly_ && (commands.size() != 1 || commands[0].name != "position"))) {
      throw std::invalid_argument("lift exports one position command only when writable");
    }
    scale_ = up_sign_ * lead / (2 * std::acos(-1));
    auto raw_params = p;
    raw_params.hardware_info.gpios.clear();
    auto & joint = raw_params.hardware_info.joints[0];
    joint.name = "lift_motor";
    joint.parameters = {{"motor_id", h.at("motor_id")}, {"feedback_id", h.at("feedback_id")}, {"model", "DM4310"},
      {"direction", "1"}, {"offset", "0"}, {"gear_ratio", "1"}, {"velocity_limit", std::to_string(speed_)},
      {"p_max", "12.5"}, {"v_max", "30"}, {"t_max", "10"}};
    joint.state_interfaces.clear(); joint.command_interfaces.clear();
    for (const auto & name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo state; state.name = name; joint.state_interfaces.push_back(state);
    }
    if (!readonly_) {
      hardware_interface::InterfaceInfo cmd; cmd.name = "velocity";
      cmd.min = std::to_string(-speed_); cmd.max = std::to_string(speed_); joint.command_interfaces.push_back(cmd);
    }
    if (motor_.on_init(raw_params) != Callback::SUCCESS) { return Callback::ERROR; }
    motor_states_ = motor_.export_state_interfaces(); motor_commands_ = motor_.export_command_interfaces();
    return Callback::SUCCESS;
  } catch (const std::exception & error) { RCLCPP_ERROR(logger, "Invalid lift configuration: %s", error.what()); return Callback::ERROR; }
}
std::vector<hardware_interface::StateInterface> LiftSystem::export_state_interfaces()
{
  return {{"lift_joint", "position", &position_}, {"lift_joint", "velocity", &velocity_},
    {"lift_status", "upper", &upper_}, {"lift_status", "lower", &lower_},
    {"lift_status", "io_online", &online_}, {"lift_status", "homed", &homed_},
    {"lift_status", "phase", &phase_state_}, {"lift_status", "motor_position", &raw_},
    {"lift_status", "motor_torque", &raw_torque_}};
}
std::vector<hardware_interface::CommandInterface> LiftSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  if (!readonly_) { result.emplace_back("lift_joint", "position", &command_); }
  return result;
}
void LiftSystem::open_io()
{
  io_fd_ = ::open(io_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (io_fd_ < 0 || flock(io_fd_, LOCK_EX | LOCK_NB) != 0) { throw std::runtime_error("cannot exclusively open lift IO"); }
  if (ioctl(io_fd_, TIOCEXCL) != 0) { throw std::runtime_error("cannot reserve lift IO tty"); }
  termios config{};
  if (tcgetattr(io_fd_, &config)) { throw std::runtime_error("cannot read lift IO termios"); }
  cfmakeraw(&config); cfsetispeed(&config, B115200); cfsetospeed(&config, B115200);
  config.c_cflag |= CLOCAL | CREAD; config.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
  config.c_cc[VMIN] = 0; config.c_cc[VTIME] = 0;
  if (tcsetattr(io_fd_, TCSANOW, &config)) { throw std::runtime_error("cannot configure lift IO"); }
}
void LiftSystem::close_io() noexcept
{ if (io_fd_ >= 0) { ioctl(io_fd_, TIOCNXCL); ::close(io_fd_); io_fd_ = -1; } }
bool LiftSystem::io_fresh() const
{
  return io_received_ != Clock::time_point{} && std::chrono::duration<double>(Clock::now()-io_received_).count() <= io_timeout_;
}
void LiftSystem::poll_io()
{
  unsigned char bytes[256];
  for (unsigned batch = 0; batch < 4; ++batch) {
    const auto size = ::read(io_fd_, bytes, sizeof(bytes));
    if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) { throw std::runtime_error("lift IO read failed"); }
    if (size <= 0) { break; }
    io_buffer_.insert(io_buffer_.end(), bytes, bytes+size);
  }
  while (io_buffer_.size() >= 7) {
    const auto * b = io_buffer_.data();
    if (b[0] == 1 && b[1] == 4 && b[2] == 2 && b[3] == 0 && b[4] <= 3 && crc(b, 5) == (b[5] | (b[6] << 8))) {
      upper_ = (b[4] & upper_mask_) ? 1 : 0; lower_ = (b[4] & lower_mask_) ? 1 : 0;
      if (upper_ && lower_) { throw std::runtime_error("both lift limit switches active"); }
      io_received_ = Clock::now(); io_buffer_.erase(io_buffer_.begin(), io_buffer_.begin()+7);
    } else { io_buffer_.erase(io_buffer_.begin()); }
  }
  online_ = io_fresh() ? 1 : 0;
}
void LiftSystem::sample()
{
  raw_ = motor_states_[0].get_optional().value(); raw_torque_ = motor_states_[2].get_optional().value();
  if (!have_raw_) { last_raw_ = raw_; zero_ = raw_; have_raw_ = true; }
  const double delta = raw_ - last_raw_;
  if (delta > wrap_/2) { turns_ -= wrap_; }
  if (delta < -wrap_/2) { turns_ += wrap_; }
  last_raw_ = raw_;
  position_ = (raw_ + turns_ - zero_) * scale_;
  velocity_ = motor_states_[1].get_optional().value() * scale_;
}
void LiftSystem::set_homed()
{
  zero_ = raw_ + turns_; position_ = command_ = 0; homed_ = 1;
  last_speed_ = 0; phase_ = Phase::Homed; phase_state_ = static_cast<int>(phase_);
  RCLCPP_INFO(logger, "Upper limit reached; software height zero established (no firmware zero write)");
}
Callback LiftSystem::on_configure(const rclcpp_lifecycle::State & state)
{
  close_io(); io_buffer_.clear(); io_received_ = {}; have_raw_ = false; turns_ = 0;
  homed_ = 0; phase_ = Phase::Unhomed; phase_state_ = 0; claimed_ = motor_claimed_ = false; homing_stop_ = false;
  try {
    open_io();
    if (motor_.on_configure(state) != Callback::SUCCESS) { close_io(); return Callback::FAILURE; }
    sample();
    const auto deadline = Clock::now() + std::chrono::duration<double>(configure_timeout_);
    while (Clock::now() < deadline) {
      poll_io();
      if (io_fresh()) {
        if (upper_ && lower_) { throw std::runtime_error("both lift limit switches active"); }
        return Callback::SUCCESS;
      }
      // Keep the motor feedback fresh while waiting for the passive IO stream.
      if (motor_.read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != Result::OK) { throw std::runtime_error("motor feedback failed during IO wait"); }
      sample(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("no valid lift IO frame before configure timeout");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Lift configure failed: %s", error.what()); motor_.on_cleanup(state); close_io(); return Callback::FAILURE;
  }
}
Result LiftSystem::claim_motor()
{
  if (motor_claimed_) { return Result::OK; }
  const auto result = motor_.perform_command_mode_switch({"lift_motor/velocity"}, {});
  motor_claimed_ = result == Result::OK;
  return result;
}
Callback LiftSystem::on_activate(const rclcpp_lifecycle::State & state)
{
  if (!io_fresh() || (upper_ && lower_) || motor_.on_activate(state) != Callback::SUCCESS) { return Callback::ERROR; }
  active_ = true; last_speed_ = 0; last_write_ = Clock::now();
  if (home_on_activate_) {
    if (upper_) { set_homed(); }
    else {
      if (claim_motor() != Result::OK) { active_ = false; return Callback::ERROR; }
      phase_ = Phase::Homing; phase_state_ = static_cast<int>(phase_); home_started_ = Clock::now();
      RCLCPP_INFO(logger, "Homing lift toward upper limit");
    }
  }
  return Callback::SUCCESS;
}
Result LiftSystem::fault(const std::string & reason)
{
  RCLCPP_ERROR(logger, "Lift hardware error: %s", reason.c_str());
  motor_.on_deactivate(rclcpp_lifecycle::State{}); active_ = claimed_ = motor_claimed_ = false;
  phase_ = Phase::Fault; phase_state_ = static_cast<int>(phase_); homed_ = 0;
  return Result::ERROR;
}
Result LiftSystem::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (phase_ == Phase::Fault) { return Result::ERROR; }
  try {
    // Reject a stalled update loop before accepting queued old IO bytes.
    if (!io_fresh()) { return fault("limit IO timeout before receive"); }
    if (motor_.read(time, period) != Result::OK) { return fault("motor read failed"); }
    sample(); poll_io();
    if (!io_fresh()) { return fault("limit IO timeout"); }
    if (upper_ && lower_) { return fault("both lift limit switches active"); }
    if (phase_ == Phase::Homing && upper_) {
      homing_stop_ = true;
      // Establish the reference only after stopping, not at a moving encoder sample.
      if (std::abs(velocity_/scale_) <= .03) { set_homed(); }
    }
    if (phase_ == Phase::Homing && std::chrono::duration<double>(Clock::now()-home_started_).count() > home_timeout_) { return fault("homing timeout"); }
    return Result::OK;
  } catch (const std::exception & error) { return fault(error.what()); }
}
Result LiftSystem::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (phase_ == Phase::Fault) { return Result::ERROR; }
  if (readonly_ || !active_ || !motor_claimed_) { return Result::OK; }
  if (!io_fresh()) { return fault("limit IO stale before command"); }
  double desired = 0;
  bool rejected = false;
  if (phase_ == Phase::Homing) { desired = homing_stop_ ? 0 : up_sign_ * home_speed_; }
  else if (claimed_) {
    // Seed the controller with the actual height, including tiny homing stop
    // overshoot. Permit interpolation back into the target range, not farther out.
    const bool inside = command_ >= minimum_ && command_ <= maximum_;
    const bool recovering = std::isfinite(startup_hold_) &&
      command_ >= std::min(minimum_, startup_hold_) && command_ <= std::max(maximum_, startup_hold_);
    rejected = !std::isfinite(command_) || (!inside && !recovering);
    if (rejected) {
      if (rejected_warn_at_ == Clock::time_point{} || Clock::now()-rejected_warn_at_ > std::chrono::seconds(2)) {
        RCLCPP_WARN(logger, "Reject height command %.6f outside [%.6f, %.6f] m; output zero velocity", command_, minimum_, maximum_);
        rejected_warn_at_ = Clock::now();
      }
    } else {
      if (inside) { startup_hold_ = std::numeric_limits<double>::quiet_NaN(); }
      const double error = command_ - position_;
      if (std::abs(error) >= tolerance_) { desired = std::clamp(kp_*error/up_sign_, -speed_, speed_); }
    }
  }
  const auto now = Clock::now();
  const double dt = std::min(.1, std::max(0.0, std::chrono::duration<double>(now-last_write_).count()));
  double speed = std::clamp(desired, last_speed_-accel_*dt, last_speed_+accel_*dt);
  // Apply physical directional stops AFTER the slew limiter: no residual speed
  // may carry the carriage into an asserted switch while ramping down.
  if (rejected || (upper_ && speed*up_sign_ > 0) || (lower_ && speed*up_sign_ < 0) ||
    (phase_ == Phase::Homing && homing_stop_) || (phase_ != Phase::Homing && !claimed_)) { speed = 0; }
  last_speed_ = speed; last_write_ = now;
  (void)motor_commands_[0].set_value(speed);
  if (motor_.write(time, period) != Result::OK) { return fault("motor write failed"); }
  return Result::OK;
}
bool LiftSystem::valid_keys(const std::vector<std::string> & keys)
{ return std::all_of(keys.begin(), keys.end(), [](const auto & key) { return key == "lift_joint/position"; }); }
Result LiftSystem::prepare_command_mode_switch(const std::vector<std::string> & start, const std::vector<std::string> & stop)
{ return !readonly_ && valid_keys(start) && valid_keys(stop) ? Result::OK : Result::ERROR; }
Result LiftSystem::perform_command_mode_switch(const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  if (prepare_command_mode_switch(start, stop) != Result::OK || !active_) { return Result::ERROR; }
  if (!start.empty()) {
    if (phase_ != Phase::Homed) { RCLCPP_ERROR(logger, "Lift controller requires completed homing"); return Result::ERROR; }
    if (claim_motor() != Result::OK) { return fault("cannot claim lift motor"); }
    startup_hold_ = command_ = position_; claimed_ = true;
  } else if (!stop.empty()) {
    claimed_ = false; last_speed_ = 0;
    if (motor_claimed_) {
      const auto result = motor_.perform_command_mode_switch({}, {"lift_motor/velocity"});
      motor_claimed_ = false;
      if (result != Result::OK) { return fault("cannot release lift motor"); }
    }
  }
  return Result::OK;
}
Callback LiftSystem::on_deactivate(const rclcpp_lifecycle::State & state)
{
  active_ = claimed_ = motor_claimed_ = false; last_speed_ = 0;
  homed_ = 0;
  if (phase_ != Phase::Fault) { phase_ = Phase::Unhomed; phase_state_ = 0; }
  return motor_.on_deactivate(state);
}
Callback LiftSystem::on_cleanup(const rclcpp_lifecycle::State & state)
{ on_deactivate(state); close_io(); homed_ = 0; return motor_.on_cleanup(state); }
Callback LiftSystem::on_shutdown(const rclcpp_lifecycle::State & state) { return on_cleanup(state); }
Callback LiftSystem::on_error(const rclcpp_lifecycle::State & state) { return on_cleanup(state); }
}  // namespace hei_hardware
PLUGINLIB_EXPORT_CLASS(hei_hardware::LiftSystem, hardware_interface::SystemInterface)
