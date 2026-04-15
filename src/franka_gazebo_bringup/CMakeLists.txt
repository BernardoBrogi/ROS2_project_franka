cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning 
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)


project(franka_gazebo_bringup)

find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)

# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install project launch files
install(
  DIRECTORY
    launch config worlds urdf meshes test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_gazebo_franka_arm_example_controller.py)
endif()

ament_package()
