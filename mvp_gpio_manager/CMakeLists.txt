cmake_minimum_required(VERSION 3.8)
project(mvp_gpio_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  std_srvs
)

include_directories(
)
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED 
  src/gpio_manager.cpp
)

ament_target_dependencies(
  ${library_name} 
  ${dependencies} 
)

add_executable(gpio_manager_node src/gpio_manager.cpp)
target_link_libraries(gpio_manager_node ${library_name} -lwiringPi)
ament_target_dependencies(gpio_manager_node ${dependencies})

install(TARGETS gpio_manager_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

##launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()