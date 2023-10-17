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

    /**
     * @brief Service which responds to requests to change gait phase
     * 
     * @param request contains the desired gait phase
     * @param response an empty acknowledgement message
     */
    void updateStateCallback(const std::shared_ptr<UpdateState::Request> request, std::shared_ptr<UpdateState::Response> response);

    /**
     * @brief Appends a new gait phase specified from the controller parameters
     * 
     * @param phaseName name of new gait phase
     * @param gaitPhases existing list of gait phases
     */
    void modifyGaitStates(const std::string &phaseName, std::vector<State> &gaitPhases);

    /** Handles the response to gait phase change requests */
    rclcpp::Service<UpdateState>::SharedPtr mUpdateStateServicePtr;

    /** List of specified gait phases */
    std::vector<State> mGaitStates;
};

}

#endif // FSM_IMPEDANCE_CONTROLLER_HPP
