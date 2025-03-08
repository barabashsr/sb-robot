cmake_minimum_required(VERSION 3.5)
project(ws_hw_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  # /usr/include/wiringPi
)

# Find wiringPi
#find_library(WIRINGPI_LIB wiringPi REQUIRED)

# Build our library
add_library(${PROJECT_NAME} SHARED
  src/sb_system.cpp
  src/ws_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# Link
# target_link_libraries(${PROJECT_NAME}
#   ${WIRINGPI_LIB}
# )

# Export plugin
pluginlib_export_plugin_description_file(
  hardware_interface 
  ws_hw_interface.xml
)

# Install
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES ws_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
