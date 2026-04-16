cmake_minimum_required(VERSION 3.8)
project(fr3_pick_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(cube_collision_object_node src/cube_collision_object_node.cpp)
ament_target_dependencies(cube_collision_object_node
rclcpp
geometry_msgs
moveit_msgs
shape_msgs
)

install(TARGETS
cube_collision_object_node
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

ament_package()