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
    RCLCPP_INFO(this->get_node()->get_logger(), "Found %s", phaseNames[i].c_str());
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

}

// Expose the controller as visible to the rest of ros2_control
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
