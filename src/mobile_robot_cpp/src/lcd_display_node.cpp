// src/lcd_display_node.cpp
//
// ROS 2 node: LCDDisplayNode (C++)
// -------------------------------------------------
// Displays selected fields from sensor_msgs::msg::JointState on a simple serial LCD.
// Behavior mirrors the Python node version in this repository.
//
// Behavior overview:
// - Subscribes to the /joint_states topic.
// - Formats up to `max_lines_` lines (default 2), each up to MAX_CHARS_PER_LINE characters.
// - Periodically (every `update_period_` seconds) sends the formatted lines over a serial
//   connection to an attached LCD microcontroller, one line per newline, plus a final
//   trailing newline.
// - Tolerates the serial device being absent: it will retry opening on each timer tick
//   or after a failed write.
//
// Parameters (declared in constructor):
// - serial_port (string): Serial device path (default: /dev/ttyACM0)
// - baudrate (int): UART baud rate (default: 115200)
// - max_lines (int): Number of LCD lines to output (default: 2)
// - update_period (double): Update interval seconds (default: 0.5)
//
// Notes:
// - Access to the last received JointState is guarded by a mutex because callbacks and
//   timers may run on different threads.
// - Serial is configured in raw mode using termios, and common baudrates are mapped.
// - This file does not change any message content; it only formats and truncates for display.
// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// STL and POSIX includes
#include <chrono>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <fcntl.h>     // open, fcntl
#include <termios.h>   // termios config
#include <unistd.h>    // read, write, close
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>     // std::strerror

using namespace std::chrono_literals;
using namespace std;

// LCD character width per line (used to truncate formatted strings)
static const size_t MAX_CHARS_PER_LINE = 16;

// The LCD display node: subscribes to joint states and periodically writes formatted
// status lines to a serial-connected LCD controller.
class LCDDisplayNode : public rclcpp::Node
{
public:
  LCDDisplayNode()
  : Node("lcd_display_node"),
    fd_(-1)
  {
    // Declare parameters (same defaults as Python node for consistency)
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<int>("max_lines", 2);
    this->declare_parameter<double>("update_period", 0.5);

    // Read parameter values once at startup
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    max_lines_ = this->get_parameter("max_lines").as_int();
    update_period_ = this->get_parameter("update_period").as_double();

    RCLCPP_INFO(this->get_logger(), "LCD node serial=%s baud=%d update_period=%.2f",
                serial_port_.c_str(), baudrate_, update_period_);

    // Try to open serial (non-fatal if it fails; we'll retry in timer)
    try_open_serial();

    // Subscribe to /joint_states and cache the latest message
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        joint_state_ = msg;
      });

    // Set up periodic timer to send updates at update_period_ seconds interval
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(update_period_),
      std::bind(&LCDDisplayNode::timer_cb, this));
  }

  ~LCDDisplayNode()
  {
    // Clean up the serial file descriptor if open
    if (fd_ != -1) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  void timer_cb()
  {
    // Retry opening serial if not currently open
    if (fd_ == -1) {
      try_open_serial();
    }
    // Snapshot the latest joint state under lock
    sensor_msgs::msg::JointState::SharedPtr js;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      js = joint_state_;
    }
    // Format and send to LCD
    auto lines = format_lines(js);
    send_serial(lines);
  }

  std::vector<std::string> format_lines(const sensor_msgs::msg::JointState::SharedPtr & js)
  {
    std::vector<std::string> lines;
    // When no data yet, show a friendly waiting message (pad to max_lines_)
    if (!js || js->name.empty()) {
      lines.push_back("waiting...");
      if (max_lines_ > 1) lines.push_back("");
      while ((int)lines.size() < max_lines_) lines.push_back("");
      return lines;
    }
    // For each line up to max_lines_, format: name:pos with 2 decimals
    for (int i = 0; i < std::min(max_lines_, (int)js->name.size()); ++i) {
      std::ostringstream ss;
      // Remove underscores from joint name to save space
      std::string nm = js->name[i];
      nm.erase(std::remove(nm.begin(), nm.end(), '_'), nm.end());
      double pos = 0.0;
      if (i < (int)js->position.size()) pos = js->position[i]; // fallback 0.0 if missing
      ss << nm << ":" << std::fixed << std::setprecision(2) << pos;
      std::string s = ss.str();
      // Truncate to LCD width
      if (s.size() > MAX_CHARS_PER_LINE) s = s.substr(0, MAX_CHARS_PER_LINE);
      lines.push_back(s);
    }
    // Pad with empty lines up to max_lines_
    while ((int)lines.size() < max_lines_) lines.push_back("");
    return lines;
  }

  void try_open_serial()
  {
    // Attempt to (re)open serial port when not already open
    if (fd_ != -1) return;
    try {
      if (std::filesystem::exists(serial_port_)) {
        // Open with read/write, no controlling TTY, non-blocking (will set blocking for writes later)
        fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ == -1) {
          RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s: %s", serial_port_.c_str(), std::strerror(errno));
          return;
        }
        // Configure baudrate and raw mode
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
          RCLCPP_WARN(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
          ::close(fd_);
          fd_ = -1;
          return;
        }
        cfmakeraw(&tty);

        // Map common baud rates to termios constants
        speed_t spd = B115200;
        switch (baudrate_) {
          case 9600: spd = B9600; break;
          case 19200: spd = B19200; break;
          case 38400: spd = B38400; break;
          case 57600: spd = B57600; break;
          case 115200: spd = B115200; break;
          default: spd = B115200; break;
        }
        cfsetispeed(&tty, spd);
        cfsetospeed(&tty, spd);

        // Typical 8N1, enable receiver, local mode
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        // Non-blocking reads with 0-0.5s timeout (not strictly used here but safe defaults)
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5; // half-second read timeout

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
          RCLCPP_WARN(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
          ::close(fd_);
          fd_ = -1;
          return;
        }
        // Make file descriptor blocking for writes
        int flags = fcntl(fd_, F_GETFL, 0);
        flags &= ~O_NONBLOCK;
        fcntl(fd_, F_SETFL, flags);
        RCLCPP_INFO(this->get_logger(), "Opened serial port %s", serial_port_.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Serial port %s does not exist yet", serial_port_.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Exception opening serial: %s", e.what());
      fd_ = -1;
    }
  }

  void send_serial(const std::vector<std::string> & lines)
  {
    // Compose payload: one line per LCD row, newline-terminated, plus trailing newline
    if (fd_ == -1) return;
    std::string payload;
    for (size_t i = 0; i < lines.size(); ++i) {
      payload += lines[i];
      payload += '\n';
    }
    // Write in a single system call; on failure, close so the next tick retries open
    ssize_t n = ::write(fd_, payload.c_str(), payload.size());
    if (n < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
    }
  }

  // members
  int fd_;                 // serial file descriptor (-1 if closed)
  std::string serial_port_; // serial device path
  int baudrate_;           // configured baud rate (e.g., 115200)
  int max_lines_;          // number of LCD lines to output
  double update_period_;   // periodic update interval (seconds)

  std::mutex mutex_;       // guards access to joint_state_
  sensor_msgs::msg::JointState::SharedPtr joint_state_; // latest joint state
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_; // subscriber handle
  rclcpp::TimerBase::SharedPtr timer_; // periodic timer
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LCDDisplayNode>();
  rclcpp::spin(node); // process callbacks until shutdown
  rclcpp::shutdown();
  return 0;
}

