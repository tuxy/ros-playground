cmake_minimum_required(VERSION 3.20)
project(warehouse_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/IDPose2D.msg"
        "srv/GetBayNumber.srv"
        DEPENDENCIES
        geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(robot_navigator src/robot_navigator.cpp)
target_include_directories(robot_navigator PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
target_link_libraries(robot_navigator PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS} ${cpp_typesupport_target})

add_executable(telemetry_monitor src/telemetry_monitor.cpp)
target_include_directories(telemetry_monitor PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
target_link_libraries(telemetry_monitor PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS} ${cpp_typesupport_target})

add_executable(fleet_manager src/fleet_manager.cpp)
target_include_directories(fleet_manager PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
target_link_libraries(fleet_manager PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${cpp_typesupport_target})

install(
        TARGETS
        robot_navigator
        telemetry_monitor
        fleet_manager
        DESTINATION
        lib/${PROJECT_NAME}
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
endif()

ament_package()
