#include "diffdrive_arduino/diffdrive_arduino_hardware.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <vector>
#include <sstream>
#include <string>
#include <cerrno>
#include <cmath>

namespace diffdrive_arduino
{

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_wheel_name_  = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];
  device_           = info_.hardware_parameters["device"];
  baudrate_         = std::stoi(info_.hardware_parameters["baudrate"]);

  left_wheel_pos_ = right_wheel_pos_ = 0.0;
  left_wheel_vel_ = right_wheel_vel_ = 0.0;
  left_wheel_cmd_ = right_wheel_cmd_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Initialized: device=%s baudrate=%d",
              device_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Reset positions cleanly on every (re)configure
  left_wheel_pos_ = right_wheel_pos_ = 0.0;
  left_wheel_vel_ = right_wheel_vel_ = 0.0;
  serial_rx_buffer_.clear();

  serial_port_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (serial_port_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "Failed to open serial port: %s — errno: %s",
                 device_.c_str(), strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty;
  if (tcgetattr(serial_port_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "tcgetattr failed: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  speed_t speed;
  switch (baudrate_) {
    case 115200: speed = B115200; break;
    case 57600:  speed = B57600;  break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                  "Unsupported baudrate %d, using 115200", baudrate_);
      speed = B115200;
      break;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag  = 0;
  tty.c_oflag  = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |=  (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "tcsetattr failed: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Serial port configured successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(left_wheel_name_,  "position", &left_wheel_pos_);
  state_interfaces.emplace_back(left_wheel_name_,  "velocity", &left_wheel_vel_);
  state_interfaces.emplace_back(right_wheel_name_, "position", &right_wheel_pos_);
  state_interfaces.emplace_back(right_wheel_name_, "velocity", &right_wheel_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(left_wheel_name_,  "velocity", &left_wheel_cmd_);
  command_interfaces.emplace_back(right_wheel_name_, "velocity", &right_wheel_cmd_);
  return command_interfaces;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  read_encoder_feedback();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveArduinoHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  send_velocity_command(left_wheel_cmd_, right_wheel_cmd_);
  return hardware_interface::return_type::OK;
}

bool DiffDriveArduinoHardware::read_encoder_feedback()
{
  char buffer[128];
  int n = ::read(serial_port_, buffer, sizeof(buffer));
  if (n <= 0) return false;

  serial_rx_buffer_.append(buffer, buffer + n);

  bool parsed_any = false;

  while (true)
  {
    auto newline_pos = serial_rx_buffer_.find('\n');
    if (newline_pos == std::string::npos) break;

    std::string line = serial_rx_buffer_.substr(0, newline_pos);
    serial_rx_buffer_.erase(0, newline_pos + 1);

    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;

    double delta_l, delta_r, vl, vr;
    if (sscanf(line.c_str(), "%lf,%lf,%lf,%lf",
               &delta_l, &delta_r, &vl, &vr) == 4)
    {
      // Guard: reject impossible deltas.
      // At 300 RPM max and 50ms cycle: max delta = 300/60 * 2π * 0.05 ≈ 1.57 rad.
      // We use 3.0 as a generous ceiling.
      const double MAX_DELTA = 3.0;
      if (std::isfinite(delta_l) && std::isfinite(delta_r) &&
          std::abs(delta_l) < MAX_DELTA && std::abs(delta_r) < MAX_DELTA)
      {
        left_wheel_pos_  += delta_l;   // accumulate delta — keeps RViz smooth
        right_wheel_pos_ += delta_r;
        left_wheel_vel_   = vl;
        right_wheel_vel_  = vr;
        parsed_any = true;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                    "Rejected bad frame: dl=%.3f dr=%.3f", delta_l, delta_r);
      }
    }
  }

  return parsed_any;
}

void DiffDriveArduinoHardware::send_velocity_command(
  double left_vel, double right_vel)
{
  char cmd[32];
  const int n = snprintf(cmd, sizeof(cmd), "%.3f,%.3f\n",
                         left_vel, right_vel);
  if (n <= 0) return;

  const ssize_t written = ::write(serial_port_, cmd,
                                  static_cast<size_t>(n));
  if (written < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "Serial write failed: %s", strerror(errno));
  }
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware,
  hardware_interface::SystemInterface)