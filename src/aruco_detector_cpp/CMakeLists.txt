cmake_minimum_required(VERSION 3.20)
project(aruco_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(detector_node src/detector_node.cpp)
target_link_libraries(
        detector_node 
        PUBLIC 
        rclcpp::rclcpp 
        ${cv_bridge_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${vision_msgs_TARGETS}
        ${message_filters_TARGETS}
        ${OpenCV_LIBS}
)

install(TARGETS detector_node DESTINATION lib/${PROJECT_NAME})

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
