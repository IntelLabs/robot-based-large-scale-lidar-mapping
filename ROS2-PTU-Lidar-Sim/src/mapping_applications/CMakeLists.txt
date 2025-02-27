cmake_minimum_required(VERSION 3.8)
project(mapping_applications)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mapping_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED) 
find_package(pcl_conversions REQUIRED) 
#find_package(PCL REQUIRED) 
find_package(tf2_sensor_msgs REQUIRED)
find_package(OpenCV 4.6.0 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(rclpy REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#World Scan Server
add_executable(world_scan_server src/world_scan_server.cpp)
ament_target_dependencies(world_scan_server
  "rclcpp"
  "mapping_interfaces"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"

)

#World Scan Client
add_executable(world_scan_client src/world_scan_client.cpp)
target_include_directories(world_scan_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(world_scan_client
  sensor_msgs
  "rclcpp"
  "mapping_interfaces"
  "geometry_msgs"
  "std_msgs"
  )

#Transform_lidar2world
add_executable(transform_lidar2world src/transform_lidar2world.cpp)
target_include_directories(transform_lidar2world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(transform_lidar2world
  "rclcpp"
  "mapping_interfaces"
  "geometry_msgs"
  "std_msgs"
  "tf2_ros"
  "sensor_msgs"
  "tf2_sensor_msgs"
  "OpenCV"
  cv_bridge
  "pcl_conversions"
  "PCL"
  )

target_link_libraries(transform_lidar2world ${PCL_LIBRARIES})

#L360 Scan Node
add_executable(l360_scan_node src/l360_scan_node.cpp)
target_include_directories(l360_scan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(l360_scan_node
  sensor_msgs
  rclcpp
  "pcl_conversions"
  "PCL"
  "mapping_interfaces"
  "sensor_msgs"
  )

  target_link_libraries(l360_scan_node ${PCL_LIBRARIES})


#Normal Compute
add_executable(compute_normal src/compute_normal.cpp)
target_include_directories(compute_normal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(compute_normal
  sensor_msgs
  rclcpp
  "pcl_conversions"
  "PCL"
  )

  target_link_libraries(compute_normal ${PCL_LIBRARIES})


#Ply Converter
add_executable(ply_converter src/ply_converter.cpp)
target_include_directories(ply_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ply_converter
  sensor_msgs
  rclcpp
  "pcl_conversions"
  "PCL"
  "mapping_interfaces"
  "geometry_msgs"
  "std_msgs"
  )

  target_link_libraries(ply_converter ${PCL_LIBRARIES})

# install(DIRECTORY include/
#         DESTINATION include)

install(TARGETS
  
  world_scan_server
  world_scan_client
  transform_lidar2world
  l360_scan_node
  compute_normal
  ply_converter

  DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  mapping_applications/scripts/pc2numpy.py
  mapping_applications/scripts/pc2numpy_low.py
  mapping_applications/scripts/pc2numpyfull.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)


ament_export_libraries(world_scan_server)

ament_package()
