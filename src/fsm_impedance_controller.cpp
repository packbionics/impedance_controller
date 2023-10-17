// Copyright (c) 2023 Pack Bionics

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "fsm_impedance_controller/fsm_impedance_controller.hpp"

namespace fsm_ic
{
controller_interface::CallbackReturn FSMImpedanceController::on_init()
{
  
  // Perform initializations for JointTrajectoryController
  controller_interface::CallbackReturn status = joint_trajectory_controller::JointTrajectoryController::on_init();
  
  // Retrieve a description of gait phases from set of parameters
  std::vector<std::string> phaseNames = this->get_node()->get_parameter("phases").as_string_array();
  for(size_t i = 0; i < phaseNames.size(); i++) {

    RCLCPP_INFO(this->get_node()->get_logger(), "Found '%s'", phaseNames[i].c_str());

    try {
      modifyGaitStates(phaseNames[i], mGaitStates);

      RCLCPP_INFO(this->get_node()->get_logger(), "Parameters for '%s':", phaseNames[i].c_str());

      RCLCPP_INFO(this->get_node()->get_logger(), "\tStiffness = '%lf'", mGaitStates.back().params.stiffness);
      RCLCPP_INFO(this->get_node()->get_logger(), "\tDamping = '%lf'", mGaitStates.back().params.damping);
      RCLCPP_INFO(this->get_node()->get_logger(), "\tEquilibrium = '%lf'", mGaitStates.back().params.equilibrium);

    } catch(const rclcpp::ParameterTypeException &e) {
      RCLCPP_ERROR(this->get_node()->get_logger(), "%s", e.what());
      RCLCPP_WARN(this->get_node()->get_logger(), "ParameterTypeException thrown while reading parameters for '%s'. Make sure parameters are given explicitly as doubles", phaseNames[i].c_str());
    }
  }

  // Create a service for updating the current gait phase
  auto updateStateCallback = std::bind(&FSMImpedanceController::updateStateCallback, this, std::placeholders::_1, std::placeholders::_2);
  mUpdateStateServicePtr = this->get_node()->create_service<UpdateState>("/fsm_impedance_controller/update_state", updateStateCallback);

  return status;
}

void FSMImpedanceController::updateStateCallback(const std::shared_ptr<UpdateState::Request> request, std::shared_ptr<UpdateState::Response> response)
{
  // TODO: Implement logic for switching between gait phases
  RCLCPP_INFO(this->get_node()->get_logger(), "State update request acknowledged.");
}

void FSMImpedanceController::modifyGaitStates(const std::string &phaseName, std::vector<State> &gaitPhases)
{

  // Create new gait phase
  State newState;

  // Extract gait phase description from parameters
  newState.params.stiffness = this->get_node()->get_parameter(phaseName + ".stiffness").as_double();
  newState.params.damping = this->get_node()->get_parameter(phaseName + ".damping").as_double();
  newState.params.equilibrium = this->get_node()->get_parameter(phaseName + ".equilibrium").as_double();

  // Append the gait phase to the list of gait phases
  // TODO: This currently results in a copy. Allocating on heap may be more efficient
  gaitPhases.push_back(newState);
}

}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
