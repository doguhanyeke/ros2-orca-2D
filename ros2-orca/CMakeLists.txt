cmake_minimum_required(VERSION 3.8)
project(ros2-orca)

# set RVO source path
set(RVO2_3D_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../RVO2-3D")
set(RVO2_2D_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../RVO2")

# Include RVO3D
add_subdirectory(
  ${RVO2_3D_PATH}
  ${CMAKE_CURRENT_BINARY_DIR}/RVO2-3D_build
)

# Include RVO2D
add_subdirectory(
  ${RVO2_2D_PATH}
  ${CMAKE_CURRENT_BINARY_DIR}/RVO2_build
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(RVO3D REQUIRED)

# Define executable
add_executable(ros2-orca src/CollisionAvoidanceSystem2D.cpp)

ament_target_dependencies(ros2-orca
  rclcpp
  std_msgs
  builtin_interfaces
  geometry_msgs
  px4_msgs
  RVO3D
)

# Link RVO2 library directly
target_link_libraries(ros2-orca
  RVO
)

# Add include directories for RVO2
target_include_directories(ros2-orca PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RVO2_2D_PATH}/src
)

# Set compiler features
target_compile_features(ros2-orca PUBLIC c_std_99 cxx_std_17)

# Installation rules
install(TARGETS ros2-orca
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
