#ifndef FSM_IMPEDANCE_CONTROLLER_HPP
#define FSM_IMPEDANCE_CONTROLLER_HPP

#include <vector>

#include <joint_trajectory_controller/joint_trajectory_controller.hpp>

#include <packbionics_interfaces/srv/update_state.hpp>

#include "fsm_impedance_controller/visibility_control.h"
#include "fsm_impedance_controller/state.hpp"

// Define typedefs for reused data types
typedef packbionics_interfaces::srv::UpdateState UpdateState;

namespace fsm_ic
{

/**
 * @brief Implementation of a finite state machine-impedance controller
 * 
 */
class FSMImpedanceController : public joint_trajectory_controller::JointTrajectoryController
{
public:

    FSM_IMPEDANCE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

private:
    void updateStateCallback(const std::shared_ptr<UpdateState::Request> request, std::shared_ptr<UpdateState::Response> response);

    rclcpp::Service<UpdateState>::SharedPtr mUpdateStateServicePtr;

    std::vector<State> gaitStates;
};

}

#endif // FSM_IMPEDANCE_CONTROLLER_HPP
