cmake_minimum_required(VERSION 3.8)
project(tcp_objectDetection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(detect_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tcp_objectDetection ${PROJECT_SOURCE_DIR}/src/tcp_objectDetection.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp)

target_include_directories(tcp_objectDetection PUBLIC ${PROJECT_SOURCE_DIR}/include)
ament_target_dependencies(tcp_objectDetection detect_msgs rclcpp sensor_msgs cv_bridge message_filters)

target_link_libraries(tcp_objectDetection ${OpenCV_LIBRARIES})

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
  tcp_objectDetection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
