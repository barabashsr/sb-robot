set(_AMENT_PACKAGE_NAME "ws_hw_interface")
set(ws_hw_interface_VERSION "0.1.0")
set(ws_hw_interface_MAINTAINER "Your Name <barabashsr@gmail.com>")
set(ws_hw_interface_BUILD_DEPENDS "rclcpp" "hardware_interface" "pluginlib")
set(ws_hw_interface_BUILDTOOL_DEPENDS "ament_cmake")
set(ws_hw_interface_BUILD_EXPORT_DEPENDS )
set(ws_hw_interface_BUILDTOOL_EXPORT_DEPENDS )
set(ws_hw_interface_EXEC_DEPENDS "rclcpp" "hardware_interface" "pluginlib" "controller_manager" "diff_drive_controller" "joint_state_broadcaster" "robot_state_publisher" "xacro")
set(ws_hw_interface_TEST_DEPENDS )
set(ws_hw_interface_GROUP_DEPENDS )
set(ws_hw_interface_MEMBER_OF_GROUPS )
set(ws_hw_interface_DEPRECATED "")
set(ws_hw_interface_EXPORT_TAGS)
list(APPEND ws_hw_interface_EXPORT_TAGS "<pluginlib plugin=\"ws_hw_interface.xml\"/>")
list(APPEND ws_hw_interface_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
