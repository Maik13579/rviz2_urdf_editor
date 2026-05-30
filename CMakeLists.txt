cmake_minimum_required(VERSION 3.8)
project(rviz2_urdf_editor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pugixml REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME} SHARED
  src/robot_state_publisher_runtime.cpp
  src/urdf_editor_state.cpp
  src/widgets.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  pugixml::pugixml
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  geometry_msgs
  pluginlib
  rclcpp
  robot_state_publisher
  rviz_common
  sensor_msgs
  std_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY layouts/
  DESTINATION share/${PROJECT_NAME}/layouts
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY test/fixtures/
  DESTINATION share/${PROJECT_NAME}/test/fixtures
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_plugin_exports
    test/test_plugin_exports.cpp
    SKIP_LINKING_MAIN_LIBRARIES
    ENV QT_QPA_PLATFORM=offscreen
  )
  if(TARGET test_plugin_exports)
    target_link_libraries(test_plugin_exports ${PROJECT_NAME} Qt5::Widgets)
    ament_target_dependencies(test_plugin_exports pluginlib rviz_common)
  endif()

  ament_add_gtest(test_panels
    test/test_panels.cpp
    SKIP_LINKING_MAIN_LIBRARIES
    ENV QT_QPA_PLATFORM=offscreen
  )
  if(TARGET test_panels)
    target_link_libraries(test_panels ${PROJECT_NAME} Qt5::Widgets)
    ament_target_dependencies(test_panels rviz_common)
  endif()

  ament_add_gtest(test_urdf_editor_state
    test/test_urdf_editor_state.cpp
    SKIP_LINKING_MAIN_LIBRARIES
    ENV
      QT_QPA_PLATFORM=offscreen
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      ROS_LOG_DIR=/tmp
  )
  if(TARGET test_urdf_editor_state)
    target_link_libraries(test_urdf_editor_state ${PROJECT_NAME} Qt5::Widgets)
    ament_target_dependencies(test_urdf_editor_state rclcpp std_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  geometry_msgs
  pluginlib
  rclcpp
  robot_state_publisher
  rviz_common
  sensor_msgs
  std_msgs
  tf2_ros
)

ament_package()
