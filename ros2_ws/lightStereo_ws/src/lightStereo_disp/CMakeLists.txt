cmake_minimum_required(VERSION 3.8)
project(lightStereo_disp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-std=c++20)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

# setup CUDA
find_package(CUDA REQUIRED)
message(STATUS "libraries: ${CUDA_LIBRARIES}")
message(STATUS "include path: ${CUDA_INCLUDE_DIRS}")
include_directories(${CUDA_INCLUDE_DIRS})

enable_language(CUDA)

# TensorRT
set(TRT_DIR "/usr/lib/aarch64-linux-gnu")
find_package(TRT_DIR)
include_directories("/usr/include/aarch64-linux-gnu")
link_directories(${TRT_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARIE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# lightStereo_disp_pub Node
add_executable(lightStereo_disp ${PROJECT_SOURCE_DIR}/src/lightStereo_disp.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp ${PROJECT_SOURCE_DIR}/src/process.cu)

ament_target_dependencies(lightStereo_disp rclcpp sensor_msgs cv_bridge message_filters)

target_link_libraries(lightStereo_disp nvinfer)
target_link_libraries(lightStereo_disp cudart)
target_link_libraries(lightStereo_disp ${OpenCV_LIBRARIES})

target_include_directories(lightStereo_disp PUBLIC ${PROJECT_SOURCE_DIR}/include)

# disp_show Node
add_executable(disp_show ${PROJECT_SOURCE_DIR}/src/disp_show.cpp)

ament_target_dependencies(disp_show rclcpp sensor_msgs cv_bridge)

target_link_libraries(disp_show ${OpenCV_LIBRARIES})

target_include_directories(disp_show PUBLIC ${PROJECT_SOURCE_DIR}/include)

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
  lightStereo_disp
  disp_show
  DESTINATION lib/${PROJECT_NAME})

ament_package()
