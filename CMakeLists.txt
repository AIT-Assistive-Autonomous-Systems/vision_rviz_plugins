cmake_minimum_required(VERSION 3.8)
project(vision_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_geometry REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_library(vision_rviz_plugins SHARED
  src/detection3_d_array_display.cpp
  src/detection3_d_array_display.hpp
  src/camera_info_display.cpp
  src/camera_info_display.hpp
  src/camera_info_visual.cpp
  src/detection_visual.cpp
  src/mesh_shape.cpp)

set_property(TARGET vision_rviz_plugins PROPERTY AUTOMOC ON)

target_include_directories(vision_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  ${Qt5Widgets_INCLUDE_DIRS})

ament_target_dependencies(
  vision_rviz_plugins
  "pluginlib"
  "rviz_common"
  "rviz_rendering"
  "sensor_msgs"
  "vision_msgs"
  "image_geometry"
)
target_link_libraries(vision_rviz_plugins Qt5::Core Qt5::Gui Qt5::Widgets)

target_compile_definitions(vision_rviz_plugins PRIVATE "VISION_RVIZ_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(vision_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS vision_rviz_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  vision_rviz_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_package()
