#include <cerrno>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>


#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class TeleopNode: public rclcpp::Node 
{
  public:
    TeleopNode()
    : Node("basic_teleop", "/hei")
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      linear_speed_ = declare_parameter<double>("linear_speed", 0.05, descriptor);
      angular_speed_ = declare_parameter<double>("angular_speed", 0.15, descriptor);
      if (!std::isfinite(linear_speed_) || linear_speed_ < 0 ||
          !std::isfinite(angular_speed_) || angular_speed_ < 0) {
        throw std::invalid_argument("linear_speed and angular_speed must be finite and nonnegative");
      }
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("base_controller/cmd_vel", 5);
      RCLCPP_INFO(
        get_logger(),
        "W/S: 前进/后退，A/D: 左/右横移，Q/E: 逆/顺时针旋转，空格: 停止，Ctrl+C: 退出");
      RCLCPP_INFO(get_logger(), "平移速度 %.3f m/s，旋转速度 %.3f rad/s", linear_speed_, angular_speed_);
    }

    void run() {
        // Disable line buffering, while preserving echo and Ctrl+C signal handling.
        // Restore the original settings on every normal return from this scope.
        struct TerminalMode {
          termios original{};
          bool changed = false;
          ~TerminalMode() {
            if (changed) {
              tcsetattr(STDIN_FILENO, TCSANOW, &original);
            }
          }
        } terminal;
        if (isatty(STDIN_FILENO)) {
          if (tcgetattr(STDIN_FILENO, &terminal.original) != 0) {
            RCLCPP_ERROR(get_logger(), "无法读取终端模式");
            return;
          }
          auto immediate = terminal.original;
          immediate.c_lflag &= ~ICANON;
          immediate.c_cc[VMIN] = 1;
          immediate.c_cc[VTIME] = 0;
          if (tcsetattr(STDIN_FILENO, TCSANOW, &immediate) != 0) {
            RCLCPP_ERROR(get_logger(), "无法设置终端模式");
            return;
          }
          terminal.changed = true;
        }
        auto lastTimestamp = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
          pollfd input{STDIN_FILENO, POLLIN, 0};

          // 最多等待 50ms，之后检查退出和速度超时。
          const int result = poll(&input, 1, 50);
          if (result < 0) {
            if (errno == EINTR) {
              continue;
            }
            RCLCPP_ERROR(get_logger(), "poll 失败");
            break;
          }

          if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            RCLCPP_ERROR(get_logger(), "终端连接断开");
            break;
          }

          if (result > 0 && (input.revents & POLLIN)) {
            char key;
            if (read(STDIN_FILENO, &key, 1) == 1) {
              handle_key(key);
              lastTimestamp = std::chrono::steady_clock::now();
            }
          }

          // 500ms 没有新按键，自动归零。
          if (std::chrono::steady_clock::now() - lastTimestamp > 500ms) {
            command_ = geometry_msgs::msg::Twist{};
          }

          if (rclcpp::ok()) {
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = this->now();
            msg.twist = command_;
            publisher_->publish(msg);
          }
        }
        // Best-effort zero on EOF/terminal errors while the ROS context is valid.
        // SIGINT shutdown still relies on the controller's independent timeout.
        if (rclcpp::ok()) {
          geometry_msgs::msg::TwistStamped stop;
          stop.header.stamp = this->now();
          publisher_->publish(stop);
        }
    }
  private:

    void handle_key(char key)
    {
      command_ = geometry_msgs::msg::Twist{};

      switch (key) {
        case 'w': case 'W': command_.linear.x = linear_speed_; break;
        case 's': case 'S': command_.linear.x = -linear_speed_; break;
        case 'a': case 'A': command_.linear.y = linear_speed_; break;
        case 'd': case 'D': command_.linear.y = -linear_speed_; break;
        case 'q': case 'Q': command_.angular.z = angular_speed_; break;
        case 'e': case 'E': command_.angular.z = -angular_speed_; break;
        case ' ': break;  // command_ was reset above: stop all axes.
        default: break;
      }
    }
    double linear_speed_{0.05}, angular_speed_{0.15};
    geometry_msgs::msg::Twist command_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TeleopNode>();
    node->run();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("basic_teleop"), "%s", error.what());
    if (rclcpp::ok()) { rclcpp::shutdown(); }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
