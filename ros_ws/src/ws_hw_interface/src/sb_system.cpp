#include "ws_hw_interface/sb_system.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <vector>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ws_hw_interface
{

// -----------------------------------------------------------
// Helper function: rad/s to encoder counts per 10ms
// -----------------------------------------------------------
// int SBRobotSystemHardware::velocityRadSecToCounts10Ms(double velocity_rad_s, double rad_per_count) const
// {
//   // Convert rad/s -> counts/s
//   //    counts/s = velocity_rad_s / rad_per_count
//   double counts_per_sec = 0.0;
//   if (!std::isnan(velocity_rad_s) && rad_per_count > 0.0) {
//     counts_per_sec = velocity_rad_s / rad_per_count;
//   }

//   // Now convert counts/s -> counts/10ms
//   //    counts/10ms = counts_per_sec * 0.01
//   double counts_per_10ms = counts_per_sec * 0.01;

//   // Round to nearest integer
//   return static_cast<int>(std::round(counts_per_10ms));
// }

// -----------------------------------------------------------
// on_init
// -----------------------------------------------------------
hardware_interface::CallbackReturn SBRobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // Must call parent's on_init() first
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load config from hardware_parameters
  try {
    cfg_.left_wheel_name      = info_.hardware_parameters.at("left_wheel_name");
    cfg_.right_wheel_name     = info_.hardware_parameters.at("right_wheel_name");
    cfg_.device_addr          = info_.hardware_parameters.at("device_address");
    cfg_.device_token         = info_.hardware_parameters.at("device_token");

    // cfg_.left_wheel_direction = std::stoi(info_.hardware_parameters.at("left_wheel_direction"));
    // cfg_.right_wheel_direction = std::stoi(info_.hardware_parameters.at("right_wheel_direction"));
    // cfg_.left_wheel_channel   = std::stoi(info_.hardware_parameters.at("left_wheel_channel"));
    // cfg_.right_wheel_channel  = std::stoi(info_.hardware_parameters.at("right_wheel_channel"));
    // cfg_.device_addr          = std::stoi(info_.hardware_parameters.at("device_addr"));
    // cfg_.motor_type           = std::stoi(info_.hardware_parameters.at("motor_type"));
    // cfg_.encoder_polarity     = std::stoi(info_.hardware_parameters.at("encoder_polarity"));
    // cfg_.enc_counts_per_rev   = std::stoi(info_.hardware_parameters.at("enc_counts_per_rev"));
    // cfg_.timeout_ms           = std::stoi(info_.hardware_parameters.at("timeout_ms"));
  }
  catch (const std::out_of_range & e)
  {
    RCLCPP_FATAL(*logger_, "Missing hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_FATAL(*logger_, "Invalid hardware parameter value: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  };

  // Setup wheels with direction
  wheel_l_.setup(cfg_.left_wheel_name);
  wheel_r_.setup(cfg_.right_wheel_name);

  // Prepare logger & clock
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("ws_hw_interface.sb_system"));
  clock_  = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  // Initialize the hardware state & command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Validate the expected 2 joints, each with 1 cmd & 2 state interfaces
  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(*logger_, "Joint '%s' has %zu command interfaces. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(*logger_, "Joint '%s' has '%s' command interface, expected '%s'.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(*logger_, "Joint '%s' has %zu state interfaces. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(*logger_, "Joint '%s' has invalid state interfaces. Must be [POSITION, VELOCITY].",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(*logger_, "ws_hw_interface on_init complete.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------
// export_state_interfaces
// -----------------------------------------------------------
std::vector<hardware_interface::StateInterface> SBRobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left wheel
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  // Right wheel
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

// -----------------------------------------------------------
// export_command_interfaces
// -----------------------------------------------------------
std::vector<hardware_interface::CommandInterface> SBRobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Left wheel
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
  // Right wheel
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

// -----------------------------------------------------------
// on_activate
// -----------------------------------------------------------
hardware_interface::CallbackReturn SBRobotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Activating SBRobotSystemHardware...");

  // Connect to WS device

  comms_.begin(cfg_.device_addr, cfg_.device_token);


  //bool ok = comms_.connect();
  if (!comms_.connect())
  {
    RCLCPP_ERROR(*logger_, "Failed to connect to WS device at %s with token %s", cfg_.device_addr.c_str(), cfg_.device_token.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!comms_.init())
  {
    RCLCPP_ERROR(*logger_, "Failed to connect to init WS device");
    return hardware_interface::CallbackReturn::ERROR;
  }

  

  // Initialize the positions, velocities, commands to 0
  wheel_l_.pos = 0.0;
  wheel_l_.vel = 0.0;
  wheel_l_.cmd = 0.0;
  wheel_r_.pos = 0.0;
  wheel_r_.vel = 0.0;
  wheel_r_.cmd = 0.0;

  RCLCPP_INFO(*logger_, "SBRobotSystemHardware successfully activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------
// on_deactivate
// -----------------------------------------------------------
hardware_interface::CallbackReturn SBRobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Deactivating SBRobotSystemHardware...");

  comms_.disconnect();

  RCLCPP_INFO(*logger_, "SBRobotSystemHardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------
// read
// -----------------------------------------------------------
hardware_interface::return_type SBRobotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{


  


  if (comms_.updateState())
  {
    wheel_l_.pos = comms_.getPoseA();
    wheel_l_.vel = comms_.getVelA();
    wheel_r_.pos = comms_.getPoseB();
    wheel_r_.vel = comms_.getVelB();
  }
  else
  {
    RCLCPP_WARN(*logger_, "Fail to update wheels state");
    return hardware_interface::return_type::ERROR;
  }


  


  // // Read encoders from the I2C device
  // std::map<int, int> encoder_values;
  // if (!comms_.read_encoder_values(encoder_values))
  // {
  //   // If read fails, log warning but keep going
  //   RCLCPP_WARN(*logger_, "Failed to read encoder values from hardware.");
  //   return hardware_interface::return_type::ERROR;
  // }

  // // Update wheels
  
  // if (true)
  // {
    
   
  // }
  // else
  // {
  //   RCLCPP_WARN(*logger_, "Encoder value for left wheel channel %d not found.", left_channel);
  //   return hardware_interface::return_type::ERROR;
  // }

  // // Right wheel
  // double old_r_pos = wheel_r_.pos;
  // int right_channel = wheel_r_.channel;
  // if (encoder_values.find(right_channel) != encoder_values.end())
  // {
  //   wheel_r_.enc = encoder_values[right_channel] * wheel_r_.direction; // Reverse direction as per requirement
  //   wheel_r_.pos = wheel_r_.calc_enc_angle();     // rad
  //   wheel_r_.vel = (wheel_r_.pos - old_r_pos) / dt;
  // }
  // else
  // {
  //   RCLCPP_WARN(*logger_, "Encoder value for right wheel channel %d not found.", right_channel);
  //   return hardware_interface::return_type::ERROR;
  // }

  return hardware_interface::return_type::OK;
}

// -----------------------------------------------------------
// write
// -----------------------------------------------------------
hardware_interface::return_type SBRobotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The commands are in wheel_l_.cmd, wheel_r_.cmd (double), in rad/s
  // Apply direction multipliers before converting to counts/10ms

  // Apply direction to commands
  // double cmd_left_directional = wheel_l_.cmd * wheel_l_.direction;
  // double cmd_right_directional = wheel_r_.cmd * wheel_r_.direction;

  // // Convert rad/s to counts/10ms
  // int speed_left = velocityRadSecToCounts10Ms(cmd_left_directional, wheel_l_.rads_per_count);
  // int speed_right = velocityRadSecToCounts10Ms(cmd_right_directional, wheel_r_.rads_per_count);

  // // Prepare a map of channel to speed
  // std::map<int, int> motor_speeds;

  // motor_speeds[wheel_l_.channel] = speed_left;
  // motor_speeds[wheel_r_.channel] = speed_right;

  // // Now pass the map to set_motor_values()
  // if (!comms_.set_motor_values(motor_speeds))
  // {
  //   RCLCPP_WARN(*logger_, "Failed to set motor values on hardware.");
  //   return hardware_interface::return_type::ERROR;
  // }

  // RCLCPP_DEBUG(*logger_, "Set motor speeds: left_channel=%d speed=%d, right_channel=%d speed=%d",
  //              wheel_l_.channel, speed_left,
  //              wheel_r_.channel, speed_right);

  return hardware_interface::return_type::OK;
}

// Export plugin
}  // namespace ws_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ws_hw_interface::SBRobotSystemHardware, hardware_interface::SystemInterface)
