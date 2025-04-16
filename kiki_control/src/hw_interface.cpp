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

CallbackReturn KikiDiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

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


  wheel_l_.setup(info_.hardware_parameters["left_wheel_name"]);
  wheel_r_.setup(info_.hardware_parameters["right_wheel_name"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
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
