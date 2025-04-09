cmake_minimum_required(VERSION 3.8)
project(csi_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# OpenCV
find_package(OpenCV REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(csi_R_publisher ./src/csi_R_publisher.cpp)
ament_target_dependencies(csi_R_publisher rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(csi_R_publisher ${OpenCV_LIBRARIES})
target_include_directories(csi_R_publisher PUBLIC ./include)

add_executable(csi_L_publisher ./src/csi_L_publisher.cpp)
ament_target_dependencies(csi_L_publisher rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(csi_L_publisher ${OpenCV_LIBRARIES})
target_include_directories(csi_L_publisher PUBLIC ./include)


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

install(TARGETS
  csi_R_publisher
  csi_L_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
