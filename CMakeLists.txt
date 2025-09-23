cmake_minimum_required(VERSION 3.8)
project(rmw_rtt_bench)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Rtt.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(ros2_latency_rtt
  src/latency_rtt.cpp
  src/latency_stats.cpp
)
ament_target_dependencies(ros2_latency_rtt rclcpp builtin_interfaces rmw)
# Link rosidl C++ typesupport (rosidl_target_interfaces is deprecated in Jazzy)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(ros2_latency_rtt "${cpp_typesupport_target}")

add_executable(rtt_pinger
  src/latency_rtt_pinger.cpp
  src/latency_stats.cpp
)
ament_target_dependencies(rtt_pinger rclcpp builtin_interfaces rmw)
target_link_libraries(rtt_pinger "${cpp_typesupport_target}")

add_executable(rtt_ponger
  src/latency_rtt_ponger.cpp
)
ament_target_dependencies(rtt_ponger rclcpp builtin_interfaces rmw)
target_link_libraries(rtt_ponger "${cpp_typesupport_target}")

install(TARGETS
  ros2_latency_rtt
  rtt_pinger
  rtt_ponger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
