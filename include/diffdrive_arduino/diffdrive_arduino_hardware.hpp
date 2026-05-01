#ifndef DIFFDRIVE_ARDUINO_HARDWARE_HPP_
#define DIFFDRIVE_ARDUINO_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace diffdrive_arduino
{

class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool read_encoder_feedback();
  void send_velocity_command(double left_vel, double right_vel);

  std::string left_wheel_name_;
  std::string right_wheel_name_;
  std::string device_;
  int baudrate_;
  int serial_port_;

  double left_wheel_pos_;
  double right_wheel_pos_;
  double left_wheel_vel_;
  double right_wheel_vel_;
  double left_wheel_cmd_;
  double right_wheel_cmd_;
};

}  // namespace diffdrive_arduino

#endif  // DIFFDRIVE_ARDUINO_HARDWARE_HPP_