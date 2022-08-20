cmake_minimum_required(VERSION 3.5)
project(copto_mix)

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



find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_auto_add_library(mix_component SHARED
 src/mix_component.cpp)

target_compile_definitions(
  mix_component
  PRIVATE "COPTO_MIX_MIX_COMPONENT_BUILDING_DLL"
)

rclcpp_components_register_nodes(
  mix_component "copto_mix::PIDComponent"
)

ament_auto_add_executable(copto_mix_node src/copto_mix_node.cpp)

install(TARGETS
  copto_mix_node
  DESTINATION lib/copto_mix
)

install(
  DIRECTORY "include/"
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()