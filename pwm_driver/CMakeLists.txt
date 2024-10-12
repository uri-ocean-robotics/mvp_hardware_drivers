cmake_minimum_required(VERSION 3.8)
project(pwm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


set(dependencies
  rclcpp
  std_msgs
)

include_directories(
  include
)
set(library_name ${PROJECT_NAME})

#Add library
add_library(${library_name} SHARED 
  include/pwm_driver.cpp
  include/PCA9685.cpp
  include/I2CPeripheral.cpp
)

ament_target_dependencies(
  ${library_name} 
  ${dependencies} 
)

#Add exe
add_executable(pwm_driver_node src/pwm_driver_node.cpp)
target_link_libraries(pwm_driver_node ${library_name} i2c)
ament_target_dependencies(pwm_driver_node ${dependencies})

# add_executable(test_node src/test_node.cpp)
# target_link_libraries(test_node ${library_name} i2c)
# ament_target_dependencies(test_node ${dependencies})


install(TARGETS pwm_driver_node ${library_name}
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


##launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()