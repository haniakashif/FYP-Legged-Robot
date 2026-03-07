cmake_minimum_required(VERSION 3.8)
project(quad_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the required ROS 2 and system packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

# 2. Tell the compiler to create a shared library (.so) from our C++ file
add_library(${PROJECT_NAME} SHARED
  src/quad_interface.cpp
)

# 3. Add the include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# 4. Link the ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# 5. Link the external system libraries (yaml-cpp)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# 6. Export the plugin XML file so ROS 2 can find our class at runtime
pluginlib_export_plugin_description_file(hardware_interface quad_interface.xml)

# 7. Install the compiled library, the headers, the plugin XML, and our config folder
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES quad_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()