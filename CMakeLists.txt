cmake_minimum_required(VERSION 3.8)
project(sim)

# C++ for the lockstep bench world, Python for everything else, one package.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dji_serial_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)
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

# YOLO's stand-in, a component for the same reason: it reads every depth frame.
add_library(detector_standin SHARED src/detector_standin.cpp)
target_compile_features(detector_standin PUBLIC cxx_std_17)
target_include_directories(detector_standin PRIVATE include)
target_link_libraries(detector_standin Eigen3::Eigen gz-transport::core gz-msgs::core)
ament_target_dependencies(detector_standin
  rclcpp rclcpp_components sensor_msgs urdf vision_msgs)
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
file(GLOB PYTHON_NODES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*)
install(PROGRAMS ${PYTHON_NODES} DESTINATION lib/${PROJECT_NAME})

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
  ament_add_pytest_test(sim_pytest test TIMEOUT 600
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV "PYTEST_ADDOPTS=-p no:anyio")
endif()

ament_package()
