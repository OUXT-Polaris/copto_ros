cmake_minimum_required(VERSION 3.5)
project(copto_quat)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(quat_component SHARED src/quat_component.cpp)

target_compile_definitions(
  quat_component 
  PRIVATE "COPTO_QUAT_QUAT_COMPONENT_BUILDING_DLL"
  )

rclcpp_components_register_nodes(
  quat_component "copto_quat::QUATComponent"
  )

ament_auto_add_executable(copto_quat_node src/copto_quat_node.cpp)

target_link_libraries(copto_quat_node quat_component Eigen3::Eigen)

ament_auto_package()