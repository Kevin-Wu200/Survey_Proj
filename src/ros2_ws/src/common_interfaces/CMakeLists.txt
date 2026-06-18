cmake_minimum_required(VERSION 3.8)
project(common_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 自定义消息定义 ---
set(msg_files
  "msg/UAVStatus.msg"
  "msg/UGVStatus.msg"
  "msg/Heartbeat.msg"
  "msg/SystemAlert.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
