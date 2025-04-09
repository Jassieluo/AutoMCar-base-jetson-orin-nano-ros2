cmake_minimum_required(VERSION 3.8)
project(smbus_cpp_base_msg_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ffdirewolfbutton_msgs REQUIRED)
find_package(ffdirewolfbutton_msgs_tools REQUIRED)
find_package(smbus_car_msgs REQUIRED)

add_executable(smbus_cpp_base_msg_sub ${PROJECT_SOURCE_DIR}/src/SmbusCppBaseMsgSub.cpp )
ament_target_dependencies(smbus_cpp_base_msg_sub ffdirewolfbutton_msgs rclcpp smbus_car_msgs)
target_include_directories(smbus_cpp_base_msg_sub PUBLIC ${PROJECT_SOURCE_DIR}/../smbus_car_msgs_tools/include)
target_link_libraries(smbus_cpp_base_msg_sub ${PROJECT_SOURCE_DIR}/../../../Joysticker_ws/FFDirewolf_ws/install/ffdirewolfbutton_msgs_tools/lib/libFFDirewolfButton_msgsTools.so)
target_link_libraries(smbus_cpp_base_msg_sub ${PROJECT_SOURCE_DIR}/../../install/smbus_car_msgs_tools/lib/libSmbusCarBaseMsgsTools.so)

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

install(TARGETS smbus_cpp_base_msg_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
