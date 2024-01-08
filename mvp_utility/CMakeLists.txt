cmake_minimum_required(VERSION 3.8)
project(mvp_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(include
  ${EIGEN3_INCLUDE_DIR}
)
set(library_name ${PROJECT_NAME})

#Add library
add_library(${library_name} SHARED 
  src/pressure_to_depth.cpp
  src/imu_ned_enu.cpp
)

ament_target_dependencies(
  ${library_name} 
  ${dependencies} 
  tf2
  tf2_ros
)

#Add exe
add_executable(pressure_to_depth_node src/pressure_to_depth_node.cpp)
target_link_libraries(pressure_to_depth_node ${library_name})
ament_target_dependencies(pressure_to_depth_node ${dependencies})

add_executable(imu_ned_enu_node src/imu_ned_enu_node.cpp)
target_link_libraries(imu_ned_enu_node ${library_name})
ament_target_dependencies(imu_ned_enu_node ${dependencies})


install(TARGETS pressure_to_depth_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS imu_ned_enu_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})


# ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

##launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()