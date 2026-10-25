cmake_minimum_required(VERSION 3.8)
project(sim)

set(SIM_RENODE_ROOT "" CACHE PATH "Renode 1.17.0 portable installation with client library")

# C++ runtime, suites and offline tools; Python supplies ROS launch glue.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(GTest REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(assimp REQUIRED)
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
set_target_properties(mcb_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(mcb_protocol PUBLIC cxx_std_17)
target_include_directories(mcb_protocol PUBLIC include)
install(TARGETS mcb_protocol
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(SIM_RENODE_ROOT)
  find_package(Threads REQUIRED)
  add_library(mcb_renode_api STATIC IMPORTED)
  set_target_properties(mcb_renode_api PROPERTIES
    IMPORTED_LOCATION "${SIM_RENODE_ROOT}/client-build/librenode_api.a"
    INTERFACE_INCLUDE_DIRECTORIES "${SIM_RENODE_ROOT}/tools/external_control_client/include")
  add_executable(mcb_type_c src/mcb_type_c.cpp src/process.cpp)
  target_compile_features(mcb_type_c PUBLIC cxx_std_17)
  target_compile_definitions(mcb_type_c PRIVATE SIM_RENODE_ROOT="${SIM_RENODE_ROOT}")
  target_include_directories(mcb_type_c PRIVATE include)
  target_link_libraries(mcb_type_c mcb_protocol mcb_renode_api Threads::Threads util ${CMAKE_DL_LIBS})
  ament_target_dependencies(mcb_type_c ament_index_cpp)
  install(TARGETS mcb_type_c DESTINATION lib/${PROJECT_NAME})
  add_executable(mcb_renode_connection_suite test/mcb/mcb_renode_connection_suite.cpp src/process.cpp)
  target_compile_features(mcb_renode_connection_suite PUBLIC cxx_std_17)
  target_compile_definitions(mcb_renode_connection_suite PRIVATE
    SIM_RENODE_ROOT="${SIM_RENODE_ROOT}" SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_include_directories(mcb_renode_connection_suite PRIVATE include)
  target_link_libraries(mcb_renode_connection_suite mcb_protocol mcb_renode_api Threads::Threads util
    ${CMAKE_DL_LIBS} GTest::gtest)
  install(TARGETS mcb_renode_connection_suite DESTINATION lib/${PROJECT_NAME})
endif()

add_library(combat src/combat.cpp src/field_geometry.cpp)
target_compile_features(combat PUBLIC cxx_std_17)
target_include_directories(combat PUBLIC include)
target_link_libraries(combat Eigen3::Eigen tinyxml2::tinyxml2)
install(TARGETS combat
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

add_library(pose_history src/pose_history.cpp)
target_compile_features(pose_history PUBLIC cxx_std_17)
target_include_directories(pose_history PUBLIC include)
target_link_libraries(pose_history PUBLIC combat Eigen3::Eigen gz-msgs::core)
install(TARGETS pose_history
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

add_library(suite_support src/suite_timing.cpp src/process.cpp src/match_scenario.cpp)
target_compile_features(suite_support PUBLIC cxx_std_17)
target_include_directories(suite_support PUBLIC include)
target_link_libraries(suite_support ${CMAKE_DL_LIBS})

add_executable(display_probe src/display_probe.cpp)
target_compile_features(display_probe PUBLIC cxx_std_17)
target_link_libraries(display_probe suite_support)
install(TARGETS display_probe DESTINATION lib/${PROJECT_NAME})

foreach(tool check_bench_log compare_runs estimation_limits shot_floors rviz_to_foxglove rf2o_quality)
  add_executable(${tool} tools/${tool}.cpp)
  target_compile_features(${tool} PUBLIC cxx_std_17)
  target_include_directories(${tool} PRIVATE include)
  target_link_libraries(${tool} nlohmann_json::nlohmann_json)
  if("${tool}" STREQUAL "rviz_to_foxglove")
    target_link_libraries(${tool} yaml-cpp)
    ament_target_dependencies(${tool} ament_index_cpp)
  elseif("${tool}" STREQUAL "rf2o_quality")
    ament_target_dependencies(${tool} rclcpp diagnostic_msgs)
  endif()
  install(TARGETS ${tool} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(simplify_urdf tools/simplify_urdf.cpp)
target_compile_features(simplify_urdf PUBLIC cxx_std_17)
target_include_directories(simplify_urdf SYSTEM PRIVATE third_party/fast_simplification)
target_link_libraries(simplify_urdf Eigen3::Eigen assimp::assimp tinyxml2::tinyxml2 yaml-cpp qhull_r)
install(TARGETS simplify_urdf DESTINATION lib/${PROJECT_NAME})

foreach(suite shot_hit estimation e2e)
  if("${suite}" STREQUAL "e2e")
    add_executable(${suite}_suite test/e2e/e2e_suite.cpp test/e2e/e2e_harness.cpp)
  else()
    add_executable(${suite}_suite test/cv/${suite}_suite.cpp)
  endif()
  target_compile_features(${suite}_suite PUBLIC cxx_std_17)
  target_include_directories(${suite}_suite PRIVATE include)
  target_link_libraries(${suite}_suite suite_support combat cv_head_aim_core
    GTest::gtest Eigen3::Eigen nlohmann_json::nlohmann_json tinyxml2::tinyxml2
    gz-transport::core gz-msgs::core)
  if("${suite}" STREQUAL "e2e")
    target_link_libraries(${suite}_suite pose_history)
  endif()
  ament_target_dependencies(${suite}_suite
    rclcpp rcl_interfaces nav_msgs std_msgs visualization_msgs dji_serial_bridge
    diagnostic_msgs tf2_ros ament_index_cpp)
  install(TARGETS ${suite}_suite DESTINATION lib/${PROJECT_NAME})
endforeach()

foreach(node target_driver point_shooter target_state_truth target_state_markers e2e_map_view head_slider_relay sim_clock)
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
  elseif("${node}" STREQUAL "e2e_map_view")
    ament_target_dependencies(${node} rclcpp nav_msgs)
  elseif("${node}" STREQUAL "sim_clock")
    ament_target_dependencies(${node} rclcpp rosgraph_msgs rosidl_typesupport_introspection_cpp)
  else()
    ament_target_dependencies(${node} rclcpp)
  endif()
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(lidar_scan src/lidar_scan.cpp)
target_compile_features(lidar_scan PUBLIC cxx_std_17)
target_include_directories(lidar_scan PRIVATE include)
ament_target_dependencies(lidar_scan rclcpp sensor_msgs)
install(TARGETS lidar_scan DESTINATION lib/${PROJECT_NAME})

add_executable(localization_suite test/localization/localization_suite.cpp)
target_compile_features(localization_suite PUBLIC cxx_std_17)
target_include_directories(localization_suite PRIVATE include)
target_link_libraries(localization_suite suite_support GTest::gtest
  gz-transport::core gz-msgs::core)
ament_target_dependencies(localization_suite rclcpp diagnostic_msgs geometry_msgs
  nav_msgs sensor_msgs rcl_interfaces std_srvs tf2_ros)
install(TARGETS localization_suite DESTINATION lib/${PROJECT_NAME})

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

add_executable(match_environment src/match_environment.cpp)
target_compile_features(match_environment PUBLIC cxx_std_17)
target_link_libraries(match_environment
  Eigen3::Eigen combat cv_head_aim_core suite_support gz-transport::core gz-msgs::core)
ament_target_dependencies(match_environment
  ament_index_cpp diagnostic_msgs geometry_msgs nav_msgs rcl_interfaces rclcpp std_msgs urdf)
install(TARGETS match_environment DESTINATION lib/${PROJECT_NAME})

add_executable(pose_emulator src/pose_emulator.cpp)
target_compile_features(pose_emulator PUBLIC cxx_std_17)
target_link_libraries(pose_emulator gz-transport::core gz-msgs::core)
ament_target_dependencies(pose_emulator
  dji_serial_bridge nav_msgs rclcpp sensor_msgs std_srvs)
install(TARGETS pose_emulator DESTINATION lib/${PROJECT_NAME})

add_executable(auto_explore src/auto_explore.cpp)
target_compile_features(auto_explore PUBLIC cxx_std_17)
target_include_directories(auto_explore PRIVATE include)
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

add_library(mcb_firmware_system SHARED src/mcb_firmware_system.cpp)
add_executable(mcb_ready src/mcb_ready.cpp)
target_compile_features(mcb_ready PUBLIC cxx_std_17)
target_include_directories(mcb_ready PRIVATE include)
target_link_libraries(mcb_ready mcb_protocol)
install(TARGETS mcb_ready DESTINATION lib/${PROJECT_NAME})
target_compile_features(mcb_firmware_system PUBLIC cxx_std_17)
target_include_directories(mcb_firmware_system PRIVATE include)
target_link_libraries(mcb_firmware_system mcb_protocol util gz-sim::core gz-plugin::register)
ament_target_dependencies(mcb_firmware_system rclcpp rcl_interfaces geometry_msgs std_msgs sensor_msgs ament_index_cpp)
add_executable(mcb_physics_suite test/mcb/mcb_physics_suite.cpp)
target_compile_features(mcb_physics_suite PUBLIC cxx_std_17)
target_link_libraries(mcb_physics_suite suite_support mcb_protocol util GTest::gtest)
target_include_directories(mcb_physics_suite PRIVATE include)
target_compile_definitions(mcb_physics_suite PRIVATE SIM_MCB_SYSTEM="$<TARGET_FILE:mcb_firmware_system>")
ament_target_dependencies(mcb_physics_suite rclcpp sensor_msgs)
install(TARGETS mcb_physics_suite DESTINATION lib/${PROJECT_NAME})
install(TARGETS mcb_firmware_system LIBRARY DESTINATION lib)

add_library(lockstep_gate SHARED src/lockstep_gate.cpp)
target_compile_features(lockstep_gate PUBLIC cxx_std_17)
target_link_libraries(lockstep_gate
  gz-sim::core gz-plugin::register gz-transport::core gz-msgs::core)
install(TARGETS lockstep_gate LIBRARY DESTINATION lib)
add_executable(lockstep_coordinator src/lockstep_coordinator.cpp)
target_compile_features(lockstep_coordinator PUBLIC cxx_std_17)
target_link_libraries(lockstep_coordinator gz-transport::core gz-msgs::core)
ament_target_dependencies(lockstep_coordinator rclcpp diagnostic_msgs std_msgs)
install(TARGETS lockstep_coordinator DESTINATION lib/${PROJECT_NAME})

# On-demand transport/physics checks; registered colcon tests launch no gz.
add_executable(lockstep_suite test/lockstep/lockstep_suite.cpp)
target_compile_features(lockstep_suite PUBLIC cxx_std_17)
target_include_directories(lockstep_suite PRIVATE include)
target_compile_definitions(lockstep_suite PRIVATE SIM_LOCKSTEP_GATE="$<TARGET_FILE:lockstep_gate>")
target_link_libraries(lockstep_suite suite_support GTest::gtest gz-transport::core gz-msgs::core)
ament_target_dependencies(lockstep_suite rclcpp diagnostic_msgs std_msgs)
install(TARGETS lockstep_suite DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch urdf meshes world rviz foxglove renode DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The image's pip anyio ships a pytest plugin that breaks apt pytest 6.2.5,
  # which fails ament's `pytest --version` probe; setup.cfg disables it for
  # the run itself, and its [tool:pytest] deselects the integration suites.
  set(ENV{PYTEST_ADDOPTS} "-p no:anyio")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_pytest_test(test_lidar_visibility test/test_lidar_visibility.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_gtest(test_lidar_scan test/cpp/test_lidar_scan.cpp)
  target_include_directories(test_lidar_scan PRIVATE include)
  target_compile_definitions(test_lidar_scan PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_lidar_scan tinyxml2::tinyxml2)
  ament_target_dependencies(test_lidar_scan sensor_msgs)
  ament_add_gtest(test_depth_mm test/cpp/test_depth_mm.cpp)
  target_include_directories(test_depth_mm PRIVATE include)
  ament_add_gtest(test_panel_view test/cpp/test_panel_view.cpp)
  target_include_directories(test_panel_view PRIVATE include)
  target_link_libraries(test_panel_view Eigen3::Eigen)
  ament_add_gtest(test_cv_head_aim test/cpp/test_cv_head_aim.cpp)
  target_include_directories(test_cv_head_aim PRIVATE include)
  target_link_libraries(test_cv_head_aim cv_head_aim_core Eigen3::Eigen)
  ament_add_gtest(test_mcb_protocol test/cpp/test_mcb_protocol.cpp)
  ament_add_gtest(test_mcb_motor test/cpp/test_mcb_motor.cpp)
  target_include_directories(test_mcb_motor PRIVATE include)
  target_link_libraries(test_mcb_protocol mcb_protocol)
  ament_add_gtest(test_combat test/cpp/test_combat.cpp)
  target_include_directories(test_combat PRIVATE include)
  target_link_libraries(test_combat combat Eigen3::Eigen)
  ament_add_gtest(test_cv_bench test/cpp/test_cv_bench.cpp)
  target_include_directories(test_cv_bench PRIVATE include)
  target_compile_definitions(test_cv_bench PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_cv_bench Eigen3::Eigen nlohmann_json::nlohmann_json tinyxml2::tinyxml2)
  target_link_libraries(test_cv_bench cv_head_aim_core)
  ament_add_gtest(test_mcb_pty_link test/cpp/test_mcb_pty_link.cpp)
  target_include_directories(test_mcb_pty_link PRIVATE include)
  ament_add_gtest(test_mcb_firmware test/cpp/test_mcb_firmware.cpp)
  target_include_directories(test_mcb_firmware PRIVATE include)
  target_link_libraries(test_mcb_firmware mcb_protocol util)
  ament_add_gtest(test_suite_timing test/cpp/test_suite_timing.cpp)
  target_link_libraries(test_suite_timing suite_support)
  ament_add_gtest(test_suite_node test/cpp/test_suite_node.cpp)
  target_include_directories(test_suite_node PRIVATE include)
  ament_target_dependencies(test_suite_node rclcpp rcl_interfaces)
  ament_add_gtest(test_e2e_referee test/cpp/test_e2e_referee.cpp test/e2e/e2e_harness.cpp)
  target_compile_features(test_e2e_referee PUBLIC cxx_std_17)
  target_include_directories(test_e2e_referee PRIVATE include)
  target_link_libraries(test_e2e_referee suite_support combat cv_head_aim_core pose_history
    Eigen3::Eigen nlohmann_json::nlohmann_json tinyxml2::tinyxml2
    gz-transport::core gz-msgs::core)
  ament_target_dependencies(test_e2e_referee
    rclcpp rcl_interfaces nav_msgs std_msgs visualization_msgs dji_serial_bridge
    diagnostic_msgs tf2_ros ament_index_cpp)
  ament_add_gtest(test_clock_bridge test/cpp/test_clock_bridge.cpp TIMEOUT 90)
  target_link_libraries(test_clock_bridge suite_support gz-transport::core gz-msgs::core)
  ament_target_dependencies(test_clock_bridge rclcpp rosgraph_msgs)
  ament_add_gtest(test_match_routes test/cpp/test_match_routes.cpp)
  target_compile_definitions(test_match_routes PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_match_routes suite_support combat tinyxml2::tinyxml2)
  ament_add_gtest(test_mcb_bridge_pty test/cpp/test_mcb_bridge_pty.cpp)
  target_compile_definitions(test_mcb_bridge_pty PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_mcb_bridge_pty suite_support mcb_protocol util)
  ament_target_dependencies(test_mcb_bridge_pty rclcpp dji_serial_bridge geometry_msgs)
  ament_add_gtest(test_suite_exit test/cpp/test_suite_exit.cpp)
  target_compile_definitions(test_suite_exit PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    SIM_CHECK_BENCH_LOG="$<TARGET_FILE:check_bench_log>")
  target_link_libraries(test_suite_exit suite_support)
  ament_add_gtest(test_foxglove_layouts test/cpp/test_foxglove_layouts.cpp)
  target_compile_definitions(test_foxglove_layouts PRIVATE SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    SIM_RVIZ_TO_FOXGLOVE="$<TARGET_FILE:rviz_to_foxglove>")
  target_link_libraries(test_foxglove_layouts suite_support)
  ament_add_gtest(test_target_odometry test/cpp/test_target_odometry.cpp)
  target_include_directories(test_target_odometry PRIVATE include)
  ament_target_dependencies(test_target_odometry builtin_interfaces dji_serial_bridge nav_msgs)
  ament_add_gtest(test_pose_history test/cpp/test_pose_history.cpp)
  target_include_directories(test_pose_history PRIVATE include)
  target_link_libraries(test_pose_history pose_history Eigen3::Eigen)
  foreach(lint copyright flake8 pep257)
    ament_add_pytest_test(sim_${lint} test/test_${lint}.py TIMEOUT 120
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      ENV "PYTEST_ADDOPTS=-p no:anyio")
  endforeach()
endif()

ament_package()
