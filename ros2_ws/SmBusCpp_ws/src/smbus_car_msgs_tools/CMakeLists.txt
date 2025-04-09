cmake_minimum_required(VERSION 3.8)
project(smbus_car_msgs_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(smbus_car_msgs REQUIRED)

add_library(SmbusCarBaseMsgsTools SHARED src/SmbusCarBaseMsgsTools.cpp include/smbus_car_msgs_tools/SmbusCarBaseMsgsTools.hpp)
target_include_directories(SmbusCarBaseMsgsTools PUBLIC include/smbus_car_msgs_tools)
ament_target_dependencies(SmbusCarBaseMsgsTools smbus_car_msgs)

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

install(TARGETS SmbusCarBaseMsgsTools
  DESTINATION lib/
)

ament_package()
