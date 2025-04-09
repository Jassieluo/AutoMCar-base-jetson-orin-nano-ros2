cmake_minimum_required(VERSION 3.8)
project(ffdirewolf_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ffdirewolfbutton_msgs REQUIRED)

message(STATUS ${ffdirewolfbutton_msgs_INCLUDE_DIRS})
message(STATUS ${ffdirewolfbutton_msgs_LIBIRARIES})

add_executable(ffdirewolf_publisher ${PROJECT_SOURCE_DIR}/src/ffdirewolf_publisher.cpp)
ament_target_dependencies(ffdirewolf_publisher ffdirewolfbutton_msgs rclcpp)
target_include_directories(ffdirewolf_publisher PUBLIC ${PROJECT_SOURCE_DIR}/../ffdirewolfbutton_msgs_tools/include)
target_link_libraries(ffdirewolf_publisher ${PROJECT_SOURCE_DIR}/../../install/ffdirewolfbutton_msgs_tools/lib/libFFDirewolfButton_msgsTools.so)

install(TARGETS ffdirewolf_publisher
  DESTINATION lib/${PROJECT_NAME}
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
