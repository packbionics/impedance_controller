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

  // Create a service for updating the current gait phase
  auto updateImpedanceCallback = std::bind(&FSMImpedanceController::updateImpedanceCallback, this, std::placeholders::_1, std::placeholders::_2);
  mUpdateImpedanceServicePtr = this->get_node()->create_service<UpdateImpedance>("/fsm_impedance_controller/update_impedance", updateImpedanceCallback);

  return status;
}

void FSMImpedanceController::updateImpedanceCallback(const std::shared_ptr<UpdateImpedance::Request> request, std::shared_ptr<UpdateImpedance::Response> response)
{
  // TODO: Implement logic for switching between gait phases
  RCLCPP_INFO(this->get_node()->get_logger(), "Impedance update request acknowledged.");
}

}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
