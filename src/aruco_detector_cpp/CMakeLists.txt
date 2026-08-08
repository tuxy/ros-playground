cmake_minimum_required(VERSION 3.20)
project(aruco_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
  add_compile_options(-O3)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(depthai REQUIRED)
find_package(Threads REQUIRED)

add_executable(detector_node src/detector_node.cpp)
target_include_directories(detector_node PRIVATE src)
target_link_libraries(
        detector_node 
        PUBLIC
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        ${aruco_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${tf2_ros_TARGETS}
        ${OpenCV_LIBS}
        depthai::core
        Threads::Threads
)

install(TARGETS detector_node RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
