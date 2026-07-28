cmake_minimum_required(VERSION 3.20)
project(aruco_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(depthai REQUIRED)

add_executable(detector_node src/detector_node.cpp)
target_link_libraries(
        detector_node
        PUBLIC
        rclcpp::rclcpp
        ${aruco_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${OpenCV_LIBS}
        depthai::core
)

install(TARGETS detector_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
