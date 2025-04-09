cmake_minimum_required(VERSION 3.8)
project(points3d_vector_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ffdirewolfbutton_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(point3D_Vector_control ${PROJECT_SOURCE_DIR}/src/point3D_Vector_control.cpp)
ament_target_dependencies(point3D_Vector_control rclcpp sensor_msgs cv_bridge ffdirewolfbutton_msgs)

target_link_libraries(point3D_Vector_control ${OpenCV_LIBRARIES})

target_include_directories(point3D_Vector_control PUBLIC ${PROJECT_SOURCE_DIR}/../../../Joysticker_ws/FFDirewolf_ws/src/ffdirewolfbutton_msgs_tools/include)
target_link_libraries(point3D_Vector_control ${PROJECT_SOURCE_DIR}/../../../Joysticker_ws/FFDirewolf_ws/install/ffdirewolfbutton_msgs_tools/lib/libFFDirewolfButton_msgsTools.so)


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
  point3D_Vector_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
