cmake_minimum_required(VERSION 3.8)
project(brain)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
# Keep the firmware's brain/msg/Kick DDS type without building the demo Brain.
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Kick.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
