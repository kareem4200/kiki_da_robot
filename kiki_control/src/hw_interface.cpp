// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hw_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kiki_diffdrive
{

// GPIO::PWM p_rm(RM_PWM, 50);
// GPIO::PWM p_lm(LM_PWM, 50);

CallbackReturn KikiDiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // const int RM_IN1 = 6; //ON BOARD 31
  // const int RM_IN2 = 22; //ON BOARD 15
  // const int RM_PWM = 13; // ON BOARD 33

  // const int LM_IN1 = 23; // ON BOARD 16
  // const int LM_IN2 = 18; // ON BOARD 12 
  // const int LM_PWM = 12; // ON BOARD 32
  // GPIO::cleanup();
  GPIO::setmode(GPIO::BCM);

  GPIO::setup(RM_IN1, GPIO::OUT, GPIO::LOW);
  GPIO::setup(RM_IN2, GPIO::OUT, GPIO::LOW);

  GPIO::setup(LM_IN1, GPIO::OUT, GPIO::LOW);
  GPIO::setup(LM_IN2, GPIO::OUT, GPIO::LOW);

  GPIO::setup(RM_PWM, GPIO::OUT, GPIO::LOW);
  GPIO::setup(LM_PWM, GPIO::OUT, GPIO::LOW);

  p_rm = std::make_unique<GPIO::PWM>(RM_PWM, 50);
  p_lm = std::make_unique<GPIO::PWM>(LM_PWM, 50);

  p_rm->start(0);
  p_lm->start(0);

  // GPIO::setup(RM_PWM, GPIO::OUT, GPIO::LOW);
  // GPIO::setup(LM_PWM, GPIO::OUT, GPIO::LOW);

  // p_rm = GPIO::PWM(RM_PWM, 50);
  // p_lm = GPIO::PWM(LM_PWM, 50);

  // cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  // cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
//   cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
//   cfg_.device = info_.hardware_parameters["device"];
//   cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
//   cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
//   cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
//   if (info_.hardware_parameters.count("pid_p") > 0)
//   {
//     cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
//     cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
//     cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
//     cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
//   }
//   else
//   {
//     RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "PID values not supplied, using defaults.");
//   }
  

  wheel_l_.setup(info_.hardware_parameters["left_wheel_name"]);
  wheel_r_.setup(info_.hardware_parameters["right_wheel_name"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KikiDiffDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KikiDiffDriveHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // if (joint.state_interfaces.size() != 2)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("KikiDiffDriveHardware"),
    //     "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
    //     joint.state_interfaces.size());
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("KikiDiffDriveHardware"),
    //     "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
    //     joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    // {
    //   RCLCPP_FATAL(
    //     rclcpp::get_logger("KikiDiffDriveHardware"),
    //     "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
    //     joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KikiDiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KikiDiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

CallbackReturn KikiDiffDriveHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Configuring ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KikiDiffDriveHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Cleaning up ...please wait...");

  GPIO::output(RM_IN1, GPIO::LOW);
	GPIO::output(RM_IN2, GPIO::LOW);

  GPIO::output(LM_IN1, GPIO::LOW);
	GPIO::output(LM_IN2, GPIO::LOW);
    
  p_rm->stop();
  p_lm->stop();
  
  GPIO::cleanup();

  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Successfully cleaned up!");

  return CallbackReturn::SUCCESS;
}


CallbackReturn KikiDiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KikiDiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("KikiDiffDriveHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type KikiDiffDriveHardware::read()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KikiDiffDriveHardware::write()
{

  if(wheel_l_.cmd > 0 && wheel_r_.cmd > 0)
  {
    GPIO::output(RM_IN1, GPIO::HIGH);
	  GPIO::output(RM_IN2, GPIO::LOW);

    GPIO::output(LM_IN1, GPIO::HIGH);
	  GPIO::output(LM_IN2, GPIO::LOW);
  }
  else if(wheel_l_.cmd < 0 && wheel_r_.cmd < 0)
  {
    GPIO::output(RM_IN1, GPIO::LOW);
	  GPIO::output(RM_IN2, GPIO::HIGH);

    GPIO::output(LM_IN1, GPIO::LOW);
	  GPIO::output(LM_IN2, GPIO::HIGH);
  }
  else if(wheel_l_.cmd < 0 && wheel_r_.cmd > 0)
  {
    GPIO::output(RM_IN1, GPIO::HIGH);
	  GPIO::output(RM_IN2, GPIO::LOW);

    GPIO::output(LM_IN1, GPIO::LOW);
	  GPIO::output(LM_IN2, GPIO::HIGH);
  }
  else if(wheel_l_.cmd > 0 && wheel_r_.cmd < 0)
  {
    GPIO::output(RM_IN1, GPIO::LOW);
	  GPIO::output(RM_IN2, GPIO::HIGH);

    GPIO::output(LM_IN1, GPIO::HIGH);
	  GPIO::output(LM_IN2, GPIO::LOW);
  }

  p_rm->start(wheel_r_.calc_pwm());
  p_lm->start(wheel_l_.calc_pwm());

  return hardware_interface::return_type::OK;
}

}  // namespace kiki_diffdrive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  kiki_diffdrive::KikiDiffDriveHardware, hardware_interface::SystemInterface)
