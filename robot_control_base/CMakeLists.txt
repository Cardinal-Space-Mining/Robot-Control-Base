cmake_minimum_required(VERSION 3.8)
project(robot_control_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_types REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)
 
# define a library target called my_value_converter_library
add_library(robot_control_base src/robot_control_base.cpp src/state_container_node.cpp)

ament_target_dependencies(robot_control_base rclcpp custom_types sensor_msgs geometry_msgs tf2_geometry_msgs tf2 message_filters)
 
# this line to exports the library
ament_export_targets(robot_control_base HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/robot_control_base
  DESTINATION include
)
 
install(
  TARGETS robot_control_base
  EXPORT robot_control_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
