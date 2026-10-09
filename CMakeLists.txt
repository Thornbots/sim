cmake_minimum_required(VERSION 3.8)
project(sim)

# C++ runtime nodes and bench world; Python remains for launch and test harnesses.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(dji_serial_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

add_executable(bench_world src/bench_world.cpp)
target_compile_features(bench_world PUBLIC cxx_std_17)
target_link_libraries(bench_world Eigen3::Eigen)
ament_target_dependencies(bench_world
  rclcpp dji_serial_bridge geometry_msgs nav_msgs rosgraph_msgs sensor_msgs std_msgs
  visualization_msgs)
install(TARGETS bench_world DESTINATION lib/${PROJECT_NAME})

add_library(cv_head_aim_core src/cv_head_aim_core.cpp)
target_compile_features(cv_head_aim_core PUBLIC cxx_std_17)
target_include_directories(cv_head_aim_core PUBLIC include)
target_link_libraries(cv_head_aim_core Eigen3::Eigen)
install(TARGETS cv_head_aim_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

add_library(mcb_protocol src/mcb_protocol.cpp)
target_compile_features(mcb_protocol PUBLIC cxx_std_17)
target_include_directories(mcb_protocol PUBLIC include)
install(TARGETS mcb_protocol
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

add_library(combat src/combat.cpp)
target_compile_features(combat PUBLIC cxx_std_17)
target_include_directories(combat PUBLIC include)
target_link_libraries(combat Eigen3::Eigen)
install(TARGETS combat
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

foreach(node target_driver point_shooter target_state_truth target_state_markers head_slider_relay sim_clock)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  if("${node}" STREQUAL "target_driver")
    target_include_directories(${node} PRIVATE include)
    ament_target_dependencies(${node} rclcpp nav_msgs)
  elseif("${node}" STREQUAL "point_shooter")
    ament_target_dependencies(${node} dji_serial_bridge geometry_msgs nav_msgs rclcpp tf2_ros)
  elseif("${node}" STREQUAL "target_state_truth")
    target_include_directories(${node} PRIVATE include)
    ament_target_dependencies(${node} dji_serial_bridge nav_msgs rclcpp)
  elseif("${node}" STREQUAL "target_state_markers")
    ament_target_dependencies(${node} dji_serial_bridge geometry_msgs rclcpp visualization_msgs)
  elseif("${node}" STREQUAL "sim_clock")
    ament_target_dependencies(${node} rclcpp rosgraph_msgs rosidl_typesupport_introspection_cpp)
  else()
    ament_target_dependencies(${node} rclcpp)
  endif()
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(cv_head_aim src/cv_head_aim.cpp)
target_compile_features(cv_head_aim PUBLIC cxx_std_17)
target_include_directories(cv_head_aim PRIVATE include)
target_link_libraries(cv_head_aim cv_head_aim_core)
ament_target_dependencies(cv_head_aim
  dji_serial_bridge geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2_ros)
install(TARGETS cv_head_aim DESTINATION lib/${PROJECT_NAME})

add_executable(wasd_teleop src/wasd_teleop.cpp)
target_compile_features(wasd_teleop PUBLIC cxx_std_17)
ament_target_dependencies(wasd_teleop geometry_msgs rclcpp)
install(TARGETS wasd_teleop DESTINATION lib/${PROJECT_NAME})

add_executable(latch_holder src/latch_holder.cpp)
target_compile_features(latch_holder PUBLIC cxx_std_17)
ament_target_dependencies(latch_holder nav_msgs rclcpp std_msgs tf2_msgs)
install(TARGETS latch_holder DESTINATION lib/${PROJECT_NAME})

add_executable(mcb_emulator_node src/mcb_emulator_node.cpp)
set_target_properties(mcb_emulator_node PROPERTIES OUTPUT_NAME mcb_emulator)
target_compile_features(mcb_emulator_node PUBLIC cxx_std_17)
target_link_libraries(mcb_emulator_node mcb_protocol util)
ament_target_dependencies(mcb_emulator_node
  geometry_msgs nav_msgs rcl_interfaces rclcpp sensor_msgs std_msgs)
install(TARGETS mcb_emulator_node DESTINATION lib/${PROJECT_NAME})

add_executable(actor_driver src/actor_driver.cpp)
target_compile_features(actor_driver PUBLIC cxx_std_17)
target_link_libraries(actor_driver gz-transport::core gz-msgs::core)
ament_target_dependencies(actor_driver nav_msgs rclcpp ros_gz_interfaces)
install(TARGETS actor_driver DESTINATION lib/${PROJECT_NAME})

add_executable(opponent_driver src/opponent_driver.cpp)
target_compile_features(opponent_driver PUBLIC cxx_std_17)
target_link_libraries(opponent_driver gz-transport::core gz-msgs::core tinyxml2::tinyxml2)
ament_target_dependencies(opponent_driver ament_index_cpp nav_msgs rclcpp)
install(TARGETS opponent_driver DESTINATION lib/${PROJECT_NAME})

add_executable(match_driver src/match_driver.cpp)
target_compile_features(match_driver PUBLIC cxx_std_17)
target_link_libraries(match_driver
  Eigen3::Eigen combat cv_head_aim_core gz-transport::core gz-msgs::core)
ament_target_dependencies(match_driver
  ament_index_cpp diagnostic_msgs geometry_msgs nav_msgs rcl_interfaces rclcpp std_msgs urdf)
install(TARGETS match_driver DESTINATION lib/${PROJECT_NAME})

add_executable(pose_emulator src/pose_emulator.cpp)
target_compile_features(pose_emulator PUBLIC cxx_std_17)
target_link_libraries(pose_emulator gz-transport::core gz-msgs::core)
ament_target_dependencies(pose_emulator
  dji_serial_bridge nav_msgs rclcpp sensor_msgs std_srvs)
install(TARGETS pose_emulator DESTINATION lib/${PROJECT_NAME})

add_executable(auto_explore src/auto_explore.cpp)
target_compile_features(auto_explore PUBLIC cxx_std_17)
target_link_libraries(auto_explore gz-transport::core gz-msgs::core)
ament_target_dependencies(auto_explore rclcpp)
install(TARGETS auto_explore DESTINATION lib/${PROJECT_NAME})

add_executable(cv_target_emulator src/cv_target_emulator.cpp)
target_compile_features(cv_target_emulator PUBLIC cxx_std_17)
target_include_directories(cv_target_emulator PRIVATE include)
target_link_libraries(cv_target_emulator Eigen3::Eigen)
ament_target_dependencies(cv_target_emulator
  dji_serial_bridge geometry_msgs nav_msgs rclcpp sensor_msgs visualization_msgs)
install(TARGETS cv_target_emulator DESTINATION lib/${PROJECT_NAME})

# A component, so it can share the camera's container with roi_depth_node.
add_library(depth_camera_emulator SHARED src/depth_camera_emulator.cpp)
target_compile_features(depth_camera_emulator PUBLIC cxx_std_17)
target_include_directories(depth_camera_emulator PRIVATE include)
ament_target_dependencies(depth_camera_emulator
  rclcpp rclcpp_components realsense2_camera_msgs sensor_msgs)
rclcpp_components_register_node(depth_camera_emulator
  PLUGIN "sim::DepthCameraEmulator"
  EXECUTABLE depth_camera_emulator_node)
install(TARGETS depth_camera_emulator
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

# The camera chain's stand-in for the match test: panels from gz truth.
add_library(detector_standin SHARED src/detector_standin.cpp)
target_compile_features(detector_standin PUBLIC cxx_std_17)
target_include_directories(detector_standin PRIVATE include)
target_link_libraries(detector_standin Eigen3::Eigen gz-transport::core gz-msgs::core)
ament_target_dependencies(detector_standin
  dji_serial_bridge geometry_msgs rclcpp rclcpp_components urdf)
rclcpp_components_register_node(detector_standin
  PLUGIN "sim::DetectorStandin"
  EXECUTABLE detector_standin_node)
install(TARGETS detector_standin
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

# A gz system on each opponent: moves it along its path every physics step.
add_library(opponent_mover SHARED src/opponent_mover.cpp)
target_compile_features(opponent_mover PUBLIC cxx_std_17)
target_link_libraries(opponent_mover
  gz-sim::core gz-plugin::register gz-transport::core gz-msgs::core)
install(TARGETS opponent_mover LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch urdf meshes world rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The image's pip anyio ships a pytest plugin that breaks apt pytest 6.2.5,
  # which fails ament's `pytest --version` probe; setup.cfg disables it for
  # the run itself, and its [tool:pytest] deselects the integration suites.
  set(ENV{PYTEST_ADDOPTS} "-p no:anyio")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_mm test/cpp/test_depth_mm.cpp)
  target_include_directories(test_depth_mm PRIVATE include)
  ament_add_gtest(test_panel_view test/cpp/test_panel_view.cpp)
  target_include_directories(test_panel_view PRIVATE include)
  target_link_libraries(test_panel_view Eigen3::Eigen)
  ament_add_gtest(test_cv_head_aim test/cpp/test_cv_head_aim.cpp)
  target_include_directories(test_cv_head_aim PRIVATE include)
  target_link_libraries(test_cv_head_aim cv_head_aim_core Eigen3::Eigen)
  ament_add_gtest(test_mcb_protocol test/cpp/test_mcb_protocol.cpp)
  target_link_libraries(test_mcb_protocol mcb_protocol)
  ament_add_gtest(test_combat test/cpp/test_combat.cpp)
  target_include_directories(test_combat PRIVATE include)
  target_link_libraries(test_combat combat Eigen3::Eigen)
  ament_add_gtest(test_target_odometry test/cpp/test_target_odometry.cpp)
  target_include_directories(test_target_odometry PRIVATE include)
  ament_target_dependencies(test_target_odometry builtin_interfaces dji_serial_bridge nav_msgs)
  ament_add_pytest_test(sim_pytest test TIMEOUT 600
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
endif()

ament_package()
