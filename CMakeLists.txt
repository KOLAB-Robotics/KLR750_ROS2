cmake_minimum_required(VERSION 3.8)
project(klr750)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Process URDF/xacro files
xacro_add_files(
  robot/klr750_robot.urdf.xacro
  INSTALL DESTINATION robot
)

# Install directories
install(DIRECTORY
  robot/
  gazebo/
  ros2_control/
  config/
  meshes/
  launch/
  worlds/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
