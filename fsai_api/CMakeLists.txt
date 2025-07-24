cmake_minimum_required(VERSION 3.5)
project(fsai_api)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
  "msg/VCU2AI.msg"
)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

# Export rosidl runtime dependencies
ament_export_dependencies(rclcpp ackermann_msgs std_msgs rosidl_default_runtime)

# Copy FS-AI_API C sources
set(FS_AI_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/FS-AI_API")

configure_file(${FS_AI_API_DIR}/FS-AI_API/fs-ai_api.c ${CMAKE_CURRENT_BINARY_DIR}/fs-ai_api.c COPYONLY)
configure_file(${FS_AI_API_DIR}/FS-AI_API/can.c ${CMAKE_CURRENT_BINARY_DIR}/can.c COPYONLY)
configure_file(${FS_AI_API_DIR}/FS-AI_API/fs-ai_api.h ${CMAKE_CURRENT_BINARY_DIR}/fs-ai_api.h COPYONLY)
configure_file(${FS_AI_API_DIR}/FS-AI_API/can.h ${CMAKE_CURRENT_BINARY_DIR}/can.h COPYONLY)

# Build FS-AI_API C library
add_library(fs_ai_api STATIC
  ${CMAKE_CURRENT_BINARY_DIR}/fs-ai_api.c
  ${CMAKE_CURRENT_BINARY_DIR}/can.c
)
target_include_directories(fs_ai_api PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Add executables
add_executable(ackermann_can src/ackermann_can.cpp)
ament_target_dependencies(ackermann_can rclcpp ackermann_msgs std_msgs)
target_link_libraries(ackermann_can fs_ai_api)

add_executable(ai2vcu_node src/ai2vcu_node.cpp)
ament_target_dependencies(ai2vcu_node rclcpp ackermann_msgs std_msgs)
target_link_libraries(ai2vcu_node fs_ai_api)

# Link generated message type support libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ackermann_can ${cpp_typesupport_target})
target_link_libraries(ai2vcu_node ${cpp_typesupport_target})
target_link_libraries(static_inspection_A ${cpp_typesupport_target})

# Python script installation
install(PROGRAMS
  scripts/wheel_speed_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  fs_ai_api
  ackermann_can
  static_inspection_A
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
