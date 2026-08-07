cmake_minimum_required(VERSION 3.16)
project(snowplow_control)

# C++17：ros2_control 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---- 查找依赖 ----
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED gazebo)

# ---- 库 ----
add_library(${PROJECT_NAME} SHARED
  src/pmsm_cascade_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
)

add_library(snowplow_dynamic_friction_plugin SHARED
  src/dynamic_friction_plugin.cpp
)

target_include_directories(snowplow_dynamic_friction_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
)

target_compile_features(snowplow_dynamic_friction_plugin PUBLIC cxx_std_17)
target_compile_options(snowplow_dynamic_friction_plugin PRIVATE ${GAZEBO_CFLAGS_OTHER})

ament_target_dependencies(snowplow_dynamic_friction_plugin
  rclcpp
  std_msgs
)

target_link_libraries(snowplow_dynamic_friction_plugin
  ${GAZEBO_LIBRARIES}
)

# 插件导出
pluginlib_export_plugin_description_file(
  controller_interface snowplow_control_plugin.xml
)

# ---- 安装 ----
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS snowplow_dynamic_friction_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES snowplow_control_plugin.xml DESTINATION share/${PROJECT_NAME}/
  RENAME plugin_description.xml
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# ---- Lint ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
