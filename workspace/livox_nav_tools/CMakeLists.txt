cmake_minimum_required(VERSION 3.8)
project(livox_nav_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(livox_custom_to_pointcloud2 src/livox_custom_to_pointcloud2.cpp)
ament_target_dependencies(livox_custom_to_pointcloud2
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

add_executable(pcd_slice_filter src/pcd_slice_filter.cpp)
target_include_directories(pcd_slice_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_slice_filter ${PCL_LIBRARIES})

install(TARGETS
  livox_custom_to_pointcloud2
  pcd_slice_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
