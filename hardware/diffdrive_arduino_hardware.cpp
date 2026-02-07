#include "diffdrive_arduino/diffdrive_arduino_hardware.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>
#include <vector>

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

  // Get parameters
  left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];
  device_ = info_.hardware_parameters["device"];
  baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

  left_wheel_pos_ = right_wheel_pos_ = 0.0;
  left_wheel_vel_ = right_wheel_vel_ = 0.0;
  left_wheel_cmd_ = right_wheel_cmd_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Initialized hardware with device: %s at %d baud",
              device_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  serial_port_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (serial_port_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "Failed to open serial port: %s", device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty;
  if (tcgetattr(serial_port_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "Failed to get terminal attributes");
    return hardware_interface::CallbackReturn::ERROR;
  }
    speed_t speed;
  switch(baudrate_) {
  case 115200: speed = B115200; break;
  case 57600:  speed = B57600; break;
  // ... other cases
  }
  cfsetospeed(&tty, speed);  // Uses B115200 constant
  cfsetispeed(&tty, speed);  // Uses B115200 constant
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveArduinoHardware"),
                 "Failed to set serial port attributes");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Serial port configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(left_wheel_name_, "position", &left_wheel_pos_);
  state_interfaces.emplace_back(left_wheel_name_, "velocity", &left_wheel_vel_);
  state_interfaces.emplace_back(right_wheel_name_, "position", &right_wheel_pos_);
  state_interfaces.emplace_back(right_wheel_name_, "velocity", &right_wheel_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(left_wheel_name_, "velocity", &left_wheel_cmd_);
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
  char buffer[64];
  int n = ::read(serial_port_, buffer, sizeof(buffer) - 1);  // ✅ use ::read
  if (n > 0)
  {
    buffer[n] = '\0';
    double left, right;
    if (sscanf(buffer, "%lf,%lf", &left, &right) == 2)
    {
      left_wheel_vel_ = left;
      right_wheel_vel_ = right;

      // integrate position roughly (20ms update rate)
      left_wheel_pos_ += left * 0.02;
      right_wheel_pos_ += right * 0.02;
      return true;
    }
  }
  return false;
}

void DiffDriveArduinoHardware::send_velocity_command(double left_vel, double right_vel)
{
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "%.3f,%.3f\n", left_vel, right_vel);
  ::write(serial_port_, cmd, strlen(cmd));   // ✅ use ::write
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
