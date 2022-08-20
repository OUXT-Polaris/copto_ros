cmake_minimum_required(VERSION 3.5)
project(copto_run)

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
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
)

add_executable(copto_run_node src/copto_run_node.cpp)

ament_target_dependencies(copto_run_node
  rclcpp
  copto_quat
  copto_pid
  copto_mix
  rclcpp_components
  )

target_link_libraries(copto_run_node Eigen3::Eigen)

install(TARGETS
  copto_run_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()