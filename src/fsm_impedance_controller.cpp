#include "fsm_impedance_controller/fsm_impedance_controller.hpp"

namespace fsm_ic
{

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fsm_ic::FSMImpedanceController,
  controller_interface::ControllerInterface
)
