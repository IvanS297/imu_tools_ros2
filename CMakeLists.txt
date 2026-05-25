cmake_minimum_required(VERSION 3.8)
project(imu_tools)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp)
find_package(tf2)

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

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

add_library(serialib src/serialib.cpp)
add_executable(imu_calibrator src/imu_calibrate.cc)
target_include_directories(serialib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(imu_calibrator rclcpp sensor_msgs std_srvs std_msgs tf2)
target_link_libraries(imu_calibrator serialib)
install(TARGETS
  imu_calibrator
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(imu_publisher src/imu_publisher.cc)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs)
target_link_libraries(imu_publisher serialib)
install(TARGETS imu_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
