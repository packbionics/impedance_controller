#include <fsm_impedance_controller/impedance_params.hpp>

/**
 * @brief Represents single phase of the gait cycle
 * 
 */
struct State
{
    double startTime;
    double endTime;

    ImpedanceParams params;
};