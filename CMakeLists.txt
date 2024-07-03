cmake_minimum_required(VERSION 3.8)
project(ddsm_cpp_py)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(run_motor src/run_motor.cpp src/ddsm115_operator_def.cpp)
ament_target_dependencies(run_motor rclcpp std_msgs)

# Install the executable
install(TARGETS run_motor
DESTINATION lib/ddsm_cpp_py)


# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Ament package configuration
ament_package()
