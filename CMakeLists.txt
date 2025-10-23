cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino LANGUAGES CXX)

# ===============================
# Compiler configuration
# ===============================
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# Dependencies
# ===============================
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ===============================
# Build library
# ===============================
add_library(${PROJECT_NAME} SHARED
  hardware/diffdrive_arduino_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# ===============================
# Export plugin
# ===============================
pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)

# ===============================
# Install
# ===============================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ===============================
# Export dependencies
# ===============================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
