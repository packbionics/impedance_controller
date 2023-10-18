# Finite State Machine-Impedance Controller

## Description

ros2_controller which applies impedance control in joint-space based
on a specified finite number of states, each with its own impedance parameters

## Dependencies

* controller_interface
* hardware_interface
* pluginlib
* rclcpp
* rclcpp_lifecycle

## Installation

1. Download source code form Github
```bash
git clone https://github.com/packbionics/fsm_impedance_controller.git
```

or 

```bash
git clone git@github.com:packbionics/fsm_impedance_controller.git
```

2. Build package within a colcon workspace
```bash
colcon build --packages-select fsm_impedance_controller
```

## Usage

Once the workspace is sourced, the controller should be visible to the rest of ros2_control.