#ifndef ROS2_CONTROL_WS_HW_INTERFACE__SB_SYSTEM_HPP_
#define ROS2_CONTROL_WS_HW_INTERFACE__SB_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/clock.hpp"

#include "ws_driver.hpp"
#include "wheel.hpp"


namespace ws_hw_interface
{

class SBRobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SBRobotSystemHardware); 

  // 1) Initialization from hardware_info
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // 2) Export state and command interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 3) Lifecycle: activate/deactivate
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // 4) read & write hardware
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // WS driver instance
  WS_Driver comms_;
  
  // Configuration
  struct Config
  {
    std::string left_wheel_name;
    std::string right_wheel_name;
    std::string left_wheel_letter;
    std::string right_wheel_letter;
    std::string device_addr;
    std::string device_token;
    // int right_wheel_channel;
    // int left_wheel_direction;
    // int right_wheel_direction;
    // int motor_type;
    // int encoder_polarity;
    // int enc_counts_per_rev;
    int timeout_ms;
  } cfg_;

  // Wheels
  Wheel wheel_l_;
  Wheel wheel_r_;

  // Optionally store hardware states & commands for each joint in arrays
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Logging
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<rclcpp::Clock> clock_;

  // Helper function to convert a velocity in rad/s to counts per 10ms
//   int velocityRadSecToCounts10Ms(double velocity_rad_s, double rad_per_count) const;
};

}  // namespace ws_hw_interface

#endif  // ROS2_CONTROL_WS_HW_INTERFACE__SB_SYSTEM_HPP_
