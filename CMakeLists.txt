cmake_minimum_required(VERSION 3.8)
project(impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  PACKAGE_DEPENDENCIES

  controller_interface
  hardware_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  packbionics_interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(controller_manager REQUIRED)

foreach(PACKAGE_DEPENDENCY IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE_DEPENDENCY} REQUIRED)
endforeach()

generate_parameter_library(
  impedance_controller_parameters
  src/impedance_controller_parameters.yaml
)

# Describe the compilation of the controller
add_library(
  ${PROJECT_NAME} SHARED
  src/impedance_controller.cpp
)

# Include necessary header files
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>
)

# Link non-ament dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  impedance_controller_parameters
)

# Specify ament dependencies
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC ${PACKAGE_DEPENDENCIES}
)

# Use pluginlib to expose the controller as visible to the rest of ros2_control
pluginlib_export_plugin_description_file(controller_interface impedance_controller.xml)

# Describe installation procedures
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS impedance_controller impedance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_load_fsm_ic test/test_load_impedance_controller.cpp)
  ament_target_dependencies(
    test_load_fsm_ic controller_manager
  )

endif()

# Define exports for the package
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${PACKAGE_DEPENDENCIES} controller_manager
)

ament_package()
