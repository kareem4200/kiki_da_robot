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

#ifndef KIKI_DIFFDRIVE_HPP__DIFFBOT_SYSTEM_HPP_
#define KIKI_DIFFDRIVE_HPP__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

#include <JetsonGPIO.h>

#include "wheel.hpp"

#define RM_IN1 6      // ON BOARD 31
#define RM_IN2 22     // ON BOARD 15
#define RM_PWM 13     // ON BOARD 33

#define LM_IN1 23     // ON BOARD 16
#define LM_IN2 18     // ON BOARD 12 
#define LM_PWM 12     // ON BOARD 32


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kiki_diffdrive
{
class KikiDiffDriveHardware : public hardware_interface::SystemInterface
{


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KikiDiffDriveHardware);

  KIKI_DIFFDRIVE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  KIKI_DIFFDRIVE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KIKI_DIFFDRIVE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  KIKI_DIFFDRIVE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  KIKI_DIFFDRIVE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  KIKI_DIFFDRIVE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  KIKI_DIFFDRIVE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  KIKI_DIFFDRIVE_PUBLIC
  hardware_interface::return_type read() override;

  KIKI_DIFFDRIVE_PUBLIC
  hardware_interface::return_type write() override;

private:

  Wheel wheel_l_;
  Wheel wheel_r_;

  std::unique_ptr<GPIO::PWM> p_rm;
  std::unique_ptr<GPIO::PWM> p_lm;

};

}  // namespace kiki_diffdrive

#endif  // KIKI_DIFFDRIVE_HPP__DIFFBOT_SYSTEM_HPP_
