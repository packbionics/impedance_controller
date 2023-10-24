// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "impedance_controller/impedance_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace impedance_controller
{
ImpedanceController::ImpedanceController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_service_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

controller_interface::CallbackReturn ImpedanceController::on_init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Create topic to receive commands
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  // Create service to receive impedance parameters
  auto srvCb = std::bind(&ImpedanceController::serviceCallback, this, std::placeholders::_1, std::placeholders::_2);

  joints_command_service_ = get_node()->create_service<SrvType>(
    "~/impedance_params", srvCb);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ImpedanceController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.joints)
  {
    for (const auto & interface : params_.state_interfaces)
    {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn ImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (impedance_params_.size() == 0)
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    double signal;

    for(size_t joint_idx = 0; joint_idx < state_interfaces_.size(); joint_idx += 2) {

      // Use the current position and velocity to compute acceleration / torque command
      double position = state_interfaces_[joint_idx].get_value();
      double velocity = state_interfaces_[joint_idx + 1].get_value();

      signal = computeCommand(position, velocity, {1.0, 1.0, 0.0});
    }

    command_interfaces_[index].set_value(signal);
  }

  return controller_interface::return_type::OK;
}

void ImpedanceController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn ImpedanceController::read_parameters()
{

  // Ensure the parameter listener is constructed
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Make sure robot joints were described
  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Make sure a command interface was described
  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Specify command interface for each controlled joint
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + params_.interface_name);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void ImpedanceController::serviceCallback(const std::shared_ptr<SrvType::Request> req, std::shared_ptr<SrvType::Response>)
{

  // Check if the request itself is properly formatted
  if(req->stiffness.size() == req->damping.size() && req->stiffness.size() == req->equilibrium.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "command was improperly formatted [%zu %zu %zu]",
      req->stiffness.size(), req->damping.size(), req->equilibrium.size()
    );
  }

  // Check if size of input matches the number of controlled joints
  if(req->stiffness.size() != joint_names_.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "command size (%zu) does not match number of interfaces (%zu)",
      req->stiffness.size(), joint_names_.size()
    );

    return;
  }

  // Allocate enough memory to fit all impedance parameters
  if(impedance_params_.size() == 0) 
  {
    impedance_params_.resize(joint_names_.size());
  }

  // Update impedance parameters
  for(size_t idx = 0; idx < req->stiffness.size(); idx++)
  {
    impedance_params_[idx].stiffness = req->stiffness[idx];
    impedance_params_[idx].damping = req->damping[idx];
    impedance_params_[idx].equilibrium = req->equilibrium[idx];
  }

  // Notify client of changes
  for(size_t idx = 0; idx < impedance_params_.size(); idx++)
  {
    if(idx == 1)
    {
      RCLCPP_INFO(get_node()->get_logger(), 
        "Updated impedance parameters: '%.3lf' '%.3lf' '%.3lf'",
        impedance_params_[idx].stiffness,
        impedance_params_[idx].damping,
        impedance_params_[idx].equilibrium
      );
    } 
    else
    {
      RCLCPP_INFO(get_node()->get_logger(), 
        "'%.3lf' '%.3lf' '%.3lf'",
        impedance_params_[idx].stiffness,
        impedance_params_[idx].damping,
        impedance_params_[idx].equilibrium
      );
    }
  }
}

double ImpedanceController::computeCommand(double position, double velocity, const ImpedanceParams &params)
{

  // Apply the simplified impedance control law to system
  double error = params.equilibrium - position;
  return params.stiffness * error - params.damping * velocity;
}

}  // namespace impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  impedance_controller::ImpedanceController, controller_interface::ControllerInterface)