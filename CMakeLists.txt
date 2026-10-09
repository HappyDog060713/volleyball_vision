cmake_minimum_required(VERSION 3.8)
project(volleyball_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mindvision/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/mindvision/lib
)

# Core vision library
add_library(${PROJECT_NAME}_core SHARED
  src/stereo_camera_driver.cpp
  src/stereo_rectifier.cpp
  src/yolo_detector.cpp
  src/stereo_triangulation.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBS}
  MVSDK
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

# Vision Node
add_executable(volleyball_vision_node
  src/vision_node.cpp
)

target_link_libraries(volleyball_vision_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(volleyball_vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

install(TARGETS
  ${PROJECT_NAME}_core
  volleyball_vision_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install 3rdparty SDK lib
install(FILES 3rdparty/mindvision/lib/libMVSDK.so
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
