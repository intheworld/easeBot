#include "hei_hardware/damiao_system.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>
#include "pluginlib/class_list_macros.hpp"

namespace hei_hardware
{
namespace
{
using Result = hardware_interface::return_type;
using Callback = hardware_interface::CallbackReturn;
const auto logger = rclcpp::get_logger("hei_hardware.damiao");
using Parameters = std::unordered_map<std::string, std::string>;

std::string required(const Parameters & p, const std::string & key)
{
  auto found = p.find(key);
  if (found == p.end() || found->second.empty()) { throw std::invalid_argument("missing parameter: " + key); }
  return found->second;
}
std::string optional(const Parameters & p, const std::string & key, const std::string & fallback)
{
  auto found = p.find(key);
  return found == p.end() ? fallback : found->second;
}
double number(const std::string & text)
{
  std::size_t used;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) { throw std::invalid_argument("invalid finite number: " + text); }
  return value;
}
unsigned integer(const std::string & text, unsigned low, unsigned high)
{
  std::size_t used;
  const auto value = std::stoul(text, &used, 0);
  if (used != text.size() || value < low || value > high) { throw std::invalid_argument("integer out of range: " + text); }
  return static_cast<unsigned>(value);
}
}  // namespace

DamiaoSystem::~DamiaoSystem() { stop_owned(); close(); }
void DamiaoSystem::close() noexcept
{
  hei_mb_close(backend_);
  backend_ = nullptr;
}
void DamiaoSystem::check(int result)
{
  if (result != 0) { throw std::runtime_error(hei_mb_error()); }
}

Callback DamiaoSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != Callback::SUCCESS) { return Callback::ERROR; }
  try {
    const auto & p = info_.hardware_parameters;
    device_ = required(p, "port");
    if (device_[0] != '/') { throw std::invalid_argument("port must be an absolute device path"); }
    const auto readonly = optional(p, "read_only", "true");
    if (readonly != "true" && readonly != "false") { throw std::invalid_argument("read_only must be true or false"); }
    read_only_ = readonly == "true";
    baud_ = integer(optional(p, "baud", "921600"), 115200, 921600);
    if (baud_ != 115200 && baud_ != 921600) { throw std::invalid_argument("unsupported serial baud"); }
    feedback_timeout_ = std::chrono::milliseconds(integer(optional(p, "feedback_timeout_ms", "250"), 20, 2000));
    configure_timeout_ = std::chrono::milliseconds(integer(optional(p, "configure_timeout_ms", "2000"), 20, 10000));
    if ((info_.joints.size() + info_.gpios.size()) == 0 ||
      info_.joints.size() + info_.gpios.size() > 15 || !info_.sensors.empty()) {
      throw std::invalid_argument("DamiaoSystem requires 1..15 motor resources and no sensors");
    }
    std::set<unsigned> ids, feedback_ids;
    std::set<std::string> names;
    motors_.clear();
    auto resources = info_.joints;
    resources.insert(resources.end(), info_.gpios.begin(), info_.gpios.end());
    for (const auto & joint : resources) {
      Motor m;
      m.name = joint.name;
      if (!names.insert(m.name).second) { throw std::invalid_argument("duplicate joint name"); }
      m.id = integer(required(joint.parameters, "motor_id"), 1, 15);
      m.feedback_id = integer(required(joint.parameters, "feedback_id"), 1, 0x7fe);
      if (!ids.insert(m.id).second || !feedback_ids.insert(m.feedback_id).second) {
        throw std::invalid_argument("duplicate motor or feedback ID on one bus");
      }
      m.gripper = optional(joint.parameters, "control_mode", "auto") == "torque_pos";
      const bool gpio = std::any_of(info_.gpios.begin(), info_.gpios.end(),
        [&](const auto & g) { return g.name == m.name; });
      if (gpio != m.gripper || (m.gripper && (m.id != 7 || m.feedback_id != 0x17))) {
        throw std::invalid_argument("only gripper7 Torque_Pos may be a GPIO motor resource");
      }
      const auto model = required(joint.parameters, "model");
      if (model == "DM4310") { m.limits = {12.5, 30, 10}; }
      else if (model == "DM4340") { m.limits = {12.5, 10, 28}; }
      else { throw std::invalid_argument("supported models: DM4310, DM4340"); }
      m.limits.position = number(optional(joint.parameters, "p_max", std::to_string(m.limits.position)));
      m.limits.velocity = number(optional(joint.parameters, "v_max", std::to_string(m.limits.velocity)));
      m.limits.torque = number(optional(joint.parameters, "t_max", std::to_string(m.limits.torque)));
      m.direction = number(required(joint.parameters, "direction"));
      m.offset = number(required(joint.parameters, "offset"));
      m.gear = number(required(joint.parameters, "gear_ratio"));
      const auto max_float = std::numeric_limits<float>::max();
      if ((m.direction != 1 && m.direction != -1) || m.gear <= 0 || m.gear > max_float ||
        std::abs(m.offset) > max_float ||
        m.limits.position <= 0 || m.limits.position > max_float ||
        m.limits.velocity <= 0 || m.limits.velocity > max_float ||
        m.limits.torque <= 0 || m.limits.torque > max_float) {
        throw std::invalid_argument("invalid direction, gear ratio or protocol range");
      }
      if (m.gripper && (model != "DM4310" || m.direction != 1 || m.gear != 1 || m.offset != 0)) {
        throw std::invalid_argument("gripper GPIO uses untransformed DM4310 motor units");
      }
      std::set<std::string> states;
      for (const auto & state : joint.state_interfaces) {
        if ((state.name != "position" && state.name != "velocity" && state.name != "effort" && !(m.gripper && (state.name == "enabled" || state.name == "feedback_sequence"))) ||
          !states.insert(state.name).second) { throw std::invalid_argument("unsupported/duplicate state interface"); }
      }
      if (!states.count("position") || !states.count("velocity")) {
        throw std::invalid_argument("position and velocity state interfaces are required");
      }
      m.export_effort = states.count("effort");
      if (m.gripper && (!m.export_effort || !states.count("enabled") || !states.count("feedback_sequence"))) {
        throw std::invalid_argument("gripper requires effort and enabled states");
      }
      m.read_only = read_only_ || joint.command_interfaces.empty();
      if (m.read_only) {
        if (!joint.command_interfaces.empty()) { throw std::invalid_argument("read-only hardware cannot export command interfaces"); }
      } else {
        if (joint.command_interfaces.size() != (m.gripper ? 4u : 1u)) { throw std::invalid_argument("invalid number of motor command interfaces"); }
        std::set<std::string> commands;
        for (const auto & interface : joint.command_interfaces) { commands.insert(interface.name); }
        if (m.gripper && commands != std::set<std::string>{"position", "velocity", "current_ratio", "enabled"}) {
          throw std::invalid_argument("gripper needs position, velocity, current_ratio, enabled commands");
        }
        const auto & command = joint.command_interfaces[0];
        m.command_interface = command.name;
        if (command.name != "position" && command.name != "velocity") {
          throw std::invalid_argument("only position (POS_VEL) and velocity (VEL) commands are supported");
        }
        m.minimum = number(command.min);
        m.maximum = number(command.max);
        m.speed = number(required(joint.parameters, "velocity_limit"));
        if (m.gripper) {
          m.current_limit = number(optional(joint.parameters, "current_ratio_limit", "0.01"));
          m.torque_limit = number(required(joint.parameters, "torque_limit_nm"));
          if (command.name != "position" || m.minimum < -4.5 || m.maximum > 0 ||
            m.speed > 10 || m.current_limit < 0.0001 || m.current_limit > 0.05 || m.torque_limit <= 0 || m.torque_limit > m.limits.torque) {
            throw std::invalid_argument("invalid gripper position/speed/current/torque limits");
          }
          m.speed_command = m.speed;
          m.current_command = m.current_limit;
        }
        if (m.minimum >= m.maximum || m.speed <= 0 || m.speed * m.gear > m.limits.velocity) {
          throw std::invalid_argument("invalid command limits or motor speed limit");
        }
        if (command.name == "position") {
          for (double endpoint : {m.minimum, m.maximum}) {
            if (std::abs(endpoint * m.direction * m.gear + m.offset) > m.limits.position) {
              throw std::invalid_argument("joint position limits exceed motor protocol range");
            }
          }
        } else if (m.minimum >= 0 || m.maximum <= 0 ||
          std::max(std::abs(m.minimum), std::abs(m.maximum)) > m.speed) {
          throw std::invalid_argument("velocity limits must include zero and fit velocity_limit");
        }
      }
      m.expected_mode = m.gripper ? 4 : m.command_interface == "position" ? 2 : 3;
      m.index = motors_.size();
      motors_.push_back(m);
    }
    return Callback::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Invalid Damiao configuration: %s", error.what());
    return Callback::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> DamiaoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & m : motors_) {
    interfaces.emplace_back(m.name, "position", &m.position);
    interfaces.emplace_back(m.name, "velocity", &m.velocity);
    if (m.export_effort) { interfaces.emplace_back(m.name, "effort", &m.effort); }
    if (m.gripper) {
      interfaces.emplace_back(m.name, "enabled", &m.enabled_state);
      interfaces.emplace_back(m.name, "feedback_sequence", &m.sequence_state);
    }
  }
  return interfaces;
}
std::vector<hardware_interface::CommandInterface> DamiaoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (!read_only_) {
    for (auto & m : motors_) {
      if (m.read_only) { continue; }
      interfaces.emplace_back(m.name, m.command_interface, &m.command);
      if (m.gripper) {
        interfaces.emplace_back(m.name, "velocity", &m.speed_command);
        interfaces.emplace_back(m.name, "current_ratio", &m.current_command);
        interfaces.emplace_back(m.name, "enabled", &m.enable_command);
      }
    }
  }
  return interfaces;
}
bool DamiaoSystem::fresh(const Motor & m) const
{
  return m.received != Clock::time_point{} && Clock::now() - m.received <= feedback_timeout_;
}
void DamiaoSystem::poll() { check(hei_mb_refresh(backend_)); }
void DamiaoSystem::receive()
{
  check(hei_mb_poll(backend_));
  for (auto & m : motors_) {
    HeiMotorbridgeState state{};
    check(hei_mb_state(backend_, m.index, &state));
    const auto now = Clock::now();
    if (state.sequence != m.sequence) {
      m.position = (state.position - m.offset) * m.direction / m.gear;
      m.velocity = state.velocity * m.direction / m.gear;
      m.effort = state.torque * m.direction * m.gear;
      m.status = state.status;
      m.enabled_state = state.status;
      m.received = now - std::chrono::nanoseconds(state.age_ns);
      m.sequence = state.sequence;
      m.sequence_state = static_cast<double>(state.sequence);
    }
    m.fault_status = state.fault_status;
    m.mode_changed = state.mode_changed;
    m.mode = state.mode;
    if (state.mode != -1) {
      m.mode_received = now - std::chrono::nanoseconds(state.mode_age_ns);
    }
  }
}
Callback DamiaoSystem::on_configure(const rclcpp_lifecycle::State &)
{
  stop_owned();
  close();
  configured_ = active_ = false;
  faulted_ = false;

  for (auto & m : motors_) {
    m.received = m.mode_received = {};
    m.position = m.velocity = m.effort = m.command = NaN;
    m.mode = -1;
    m.sequence = 0;
    m.mode_changed = false;
    m.fault_status = 0;
    m.enable_confirmed = false;
    m.owned = false;
    m.claimed = false;
    m.enable_command = m.enabled_state = 0;
  }
  try {
    backend_ = hei_mb_open(device_.c_str(), baud_);
    if (!backend_) { throw std::runtime_error(hei_mb_error()); }
    for (const auto & m : motors_) {
      check(hei_mb_add(backend_, m.id, m.feedback_id,
        m.limits.position, m.limits.velocity, m.limits.torque,
        m.read_only ? 0 : m.expected_mode));
    }
    const auto deadline = Clock::now() + configure_timeout_;
    while (Clock::now() < deadline) {
      poll();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      receive();
      bool ready = true;
      for (const auto & m : motors_) {
        ready &= fresh(m);
        if (fresh(m) && m.fault_status > 1) { throw std::runtime_error(m.name + ": motor fault " + std::to_string(m.fault_status)); }
        if (!m.read_only) {
          ready &= m.mode != -1;
          if (fresh(m) && m.status == 1) { throw std::runtime_error(m.name + ": motor already enabled by another owner"); }
          const int expected = m.expected_mode;
          if (m.gripper && fresh(m) && m.mode != -1 && m.status == 0 &&
            (m.mode != 4 || m.mode_changed)) {
            check(hei_mb_configure_gripper(backend_, m.index));
            ready = false;
            continue;
          }
          if (m.mode_changed || (m.mode != -1 && m.mode != expected)) { throw std::runtime_error(m.name + ": CTRL_MODE mismatch; configure firmware before connecting"); }
        }
      }
      if (ready) { configured_ = true; return Callback::SUCCESS; }
    }
    throw std::runtime_error("timed out waiting for all motor feedback/modes");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Damiao configure failed: %s", error.what());
    close();
    return Callback::FAILURE;
  }
}
Callback DamiaoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_ || faulted_) { return Callback::ERROR; }
  for (auto & m : motors_) {
    if (!fresh(m) || m.status > 1) { return Callback::ERROR; }
    m.command = m.command_interface == "position" ? m.position : 0;
  }
  active_ = true;
  return Callback::SUCCESS;  // Activation alone never enables a motor.
}
bool DamiaoSystem::stop_owned() noexcept
{
  bool stopped = true;
  for (auto & m : motors_) {
    m.claimed = false;
    if (m.owned) {
      try { check(hei_mb_enable(backend_, m.index, 0)); m.owned = false; }
      catch (...) { stopped = false; }
      m.enable_confirmed = false;
    }
  }
  return stopped;
}
Result DamiaoSystem::fail(const std::string & reason)
{
  faulted_ = true;
  active_ = false;
  stop_owned();
  RCLCPP_ERROR(logger, "Damiao hardware error: %s", reason.c_str());
  return Result::ERROR;
}
Callback DamiaoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (!stop_owned()) {
    faulted_ = true;
    RCLCPP_ERROR(logger, "Failed to send all Damiao disable requests");
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}
Callback DamiaoSystem::on_cleanup(const rclcpp_lifecycle::State & state)
{
  const auto result = on_deactivate(state);
  close();
  configured_ = false;
  return result;
}
Callback DamiaoSystem::on_shutdown(const rclcpp_lifecycle::State & state) { return on_cleanup(state); }
Callback DamiaoSystem::on_error(const rclcpp_lifecycle::State & state)
{
  on_cleanup(state);
  return Callback::SUCCESS;
}
Result DamiaoSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!configured_ || faulted_) { return Result::ERROR; }
  try {
    // A stalled update loop must not make old queued USB feedback fresh again.
    for (const auto & m : motors_) {
      if (!fresh(m)) { return fail(m.name + ": feedback timeout before receive"); }
    }
    receive();
    for (auto & m : motors_) {
      if (!fresh(m)) { return fail(m.name + ": feedback timeout"); }
      if (m.fault_status > 1) { return fail(m.name + ": motor fault " + std::to_string(m.fault_status)); }
      if (!m.read_only && (m.mode_changed || Clock::now() - m.mode_received > feedback_timeout_ ||
        m.mode != m.expected_mode)) { return fail(m.name + ": stale or changed CTRL_MODE"); }
      if (m.owned) {
        if (m.gripper) {
          const bool position_exceeded = m.position < m.minimum - 0.000383 || m.position > m.maximum + 0.000383;
          const bool torque_exceeded = std::abs(m.effort) > m.torque_limit;
          if (position_exceeded || torque_exceeded) {
            return fail(m.name + ": gripper feedback limit: position_exceeded=" +
              std::to_string(position_exceeded) + ", torque_exceeded=" + std::to_string(torque_exceeded) +
              ", position_rad=" + std::to_string(m.position) + ", range_rad=[" +
              std::to_string(m.minimum) + ", " + std::to_string(m.maximum) +
              "], position_margin_rad=0.000383, torque_nm=" + std::to_string(m.effort) +
              ", torque_limit_nm=" + std::to_string(m.torque_limit) +
              ", commanded_velocity_rad_s=" + std::to_string(m.speed_command) +
              ", commanded_current_ratio=" + std::to_string(m.current_command));
          }
        }
        if (m.status == 1 && m.received >= m.enabled_at) { m.enable_confirmed = true; }
        if ((m.enable_confirmed && m.status != 1) ||
          (!m.enable_confirmed && Clock::now() - m.enabled_at > feedback_timeout_)) {
          return fail(m.name + ": enable was lost or not acknowledged");
        }
      }
    }
    poll();
    return Result::OK;
  } catch (const std::exception & error) { return fail(error.what()); }
}
void DamiaoSystem::send_command(const Motor & m, double value)
{
  const bool position = m.command_interface == "position";
  const double target = value * m.direction * m.gear + (position ? m.offset : 0);
  if (m.gripper) {
    check(hei_mb_force_position(backend_, m.index, target, m.speed_command, m.current_command));
  } else {
    check(hei_mb_command(backend_, m.index, target, m.speed * m.gear, position));
  }
}
Result DamiaoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (faulted_ || !configured_) { return Result::ERROR; }
  if (read_only_ || !active_) { return Result::OK; }
  try {
    // Validate the entire batch before sending any movement command.
    for (const auto & m : motors_) {
      if (!m.claimed) { continue; }
      if (!fresh(m) || m.mode_changed || Clock::now() - m.mode_received > feedback_timeout_ ||
        m.mode != m.expected_mode ||
        !std::isfinite(m.command) || m.command < m.minimum || m.command > m.maximum) {
        return fail(m.name + ": stale feedback or invalid/out-of-limit command");
      }
      if (m.gripper && (!std::isfinite(m.speed_command) || m.speed_command < 0.01 || m.speed_command > m.speed ||
        !std::isfinite(m.current_command) || m.current_command < 0.0001 || m.current_command > m.current_limit ||
        (m.enable_command != 0 && m.enable_command != 1))) {
        return fail(m.name + ": invalid gripper speed/current/enable command");
      }
    }
    for (auto & m : motors_) {
      if (m.gripper && m.claimed) {
        if (m.enable_command == 0 && m.owned) {
          check(hei_mb_enable(backend_, m.index, 0));
          m.owned = m.enable_confirmed = false;
        } else if (m.enable_command == 1 && !m.owned) {
          if (m.status != 0) { return fail(m.name + ": unexpected enabled motor"); }
          send_command(m, std::clamp(m.position, m.minimum, m.maximum));
          m.owned = true;
          m.enabled_at = Clock::now();
          m.enable_confirmed = false;
          check(hei_mb_enable(backend_, m.index, 1));
        }
      }
      if (m.claimed && m.enable_confirmed) { send_command(m, m.command); }
    }
    return Result::OK;
  } catch (const std::exception & error) { return fail(error.what()); }
}
bool DamiaoSystem::validate_switch(const std::vector<std::string> & keys) const
{
  for (const auto & m : motors_) {
    if (!m.gripper) { continue; }
    std::set<std::string> requested;
    for (const auto & key : keys) { if (key.rfind(m.name + "/", 0) == 0) { requested.insert(key.substr(m.name.size() + 1)); } }
    if (!requested.empty() && (m.read_only || requested != std::set<std::string>{"position", "velocity", "current_ratio", "enabled"})) { return false; }
  }
  for (const auto & key : keys) {
    for (const auto & m : motors_) {
      if (!m.gripper && key.rfind(m.name + "/", 0) == 0 && (m.read_only || key != m.name + "/" + m.command_interface)) { return false; }
    }
  }
  return true;
}
Result DamiaoSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  return validate_switch(start) && validate_switch(stop) ? Result::OK : Result::ERROR;
}
Result DamiaoSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  if (!validate_switch(start) || !validate_switch(stop) || faulted_) { return Result::ERROR; }
  try {
    for (auto & m : motors_) {
      const auto key = m.name + "/" + m.command_interface;
      if (std::find(stop.begin(), stop.end(), key) != stop.end() &&
        std::find(start.begin(), start.end(), key) == start.end() && m.owned) {
        check(hei_mb_enable(backend_, m.index, 0));
        m.owned = m.claimed = false;
        m.enable_confirmed = false;
      }
      if (std::find(stop.begin(), stop.end(), key) != stop.end() && std::find(start.begin(), start.end(), key) == start.end()) {
        m.claimed = false;
        m.enable_command = 0;
      }
    }
    for (const auto & m : motors_) {
      const auto key = m.name + "/" + m.command_interface;
      if (std::find(start.begin(), start.end(), key) == start.end()) { continue; }
      if (!active_ || !fresh(m) || m.mode_changed || m.fault_status > 1 || m.status != (m.owned ? 1 : 0) ||
        m.mode != m.expected_mode ||
        Clock::now() - m.mode_received > feedback_timeout_ ||
        (m.command_interface == "position" &&
          (m.position < m.minimum - (m.gripper ? 0.000383 : 0) || m.position > m.maximum + (m.gripper ? 0.000383 : 0)))) {
        return fail(m.name + ": motor is not ready to be claimed");
      }
    }
    for (auto & m : motors_) {
      const auto key = m.name + "/" + m.command_interface;
      if (std::find(start.begin(), start.end(), key) == start.end()) { continue; }
      m.command = m.command_interface == "position" ? m.position : 0;
      if (m.gripper) {
        m.command = std::clamp(m.position, m.minimum, m.maximum);
        m.enable_command = 0;
        m.speed_command = m.speed;
        m.current_command = m.current_limit;
        m.claimed = true;
        continue;  // Controller activation does not enable the gripper.
      }
      if (m.owned) { continue; }
      send_command(m, m.command);  // Seed hold/zero target while disabled.
      m.owned = true;  // Also attempt disable if the enable write is ambiguous.
      m.enabled_at = Clock::now();
      m.enable_confirmed = false;
      check(hei_mb_enable(backend_, m.index, 1));
      m.claimed = true;
    }
    return Result::OK;
  } catch (const std::exception & error) { return fail(error.what()); }
}
}  // namespace hei_hardware

PLUGINLIB_EXPORT_CLASS(hei_hardware::DamiaoSystem, hardware_interface::SystemInterface)
