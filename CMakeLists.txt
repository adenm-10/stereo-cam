cmake_minimum_required(VERSION 3.8)
project(stereo_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(Threads          REQUIRED)   # replaces raw ‘pthread’

# libcamera
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Include directories
include_directories(
  include
  ${LIBCAMERA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# cv_cam library
add_library(cv_cam SHARED
  src/cv_cam/cv_cam.cpp
)

target_include_directories(cv_cam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(cv_cam PUBLIC
  PkgConfig::LIBCAMERA
  ${OpenCV_LIBS}
  Threads::Threads
)

# stereo_node executable
add_library(stereo_node SHARED
  src/stereo_node.cpp
  src/calib_utils.cpp
)

target_include_directories(stereo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stereo_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
  message_filters
  OpenCV
  ament_index_cpp
  tf2
  tf2_ros
)

target_link_libraries(stereo_node
  cv_cam
)

rclcpp_components_register_nodes(stereo_node
  "stereo_cam::StereoNode")

install(TARGETS
  stereo_node
  cv_cam
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib     # or lib/${PROJECT_NAME} – just be consistent
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
)

# depth node composable node
add_library(depth_node_component SHARED
  src/depth_node.cpp
)
target_include_directories(depth_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(depth_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  stereo_msgs
  image_transport
  message_filters
  cv_bridge
)
target_link_libraries(depth_node_component
  ${OpenCV_LIBRARIES}
)

# Register component with rclcpp_components
rclcpp_components_register_node(depth_node_component
  PLUGIN "stereo_cam::DepthNode"
  EXECUTABLE depth_node_exe
)

install(
  TARGETS depth_node_component depth_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# stereo_calib executable
add_executable(stereo_calib 
  src/stereo_calib.cpp
)

ament_target_dependencies(stereo_calib
  OpenCV
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  ament_index_cpp
)

# Install targets
install(TARGETS
  stereo_calib
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install assets
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/cap.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export all libraries and includes
ament_export_include_directories(include)
ament_export_libraries(
  cv_cam
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
  camera_info_manager
  tf2
  tf2_ros
  OpenCV
)

ament_package()
