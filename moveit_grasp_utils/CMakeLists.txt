cmake_minimum_required(VERSION 3.22)
project(moveit_grasp_utils)

# ── compiler settings ──────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── dependencies ──────────────────────────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs   REQUIRED)

# ── executable ────────────────────────────────────────────────────────────
add_executable(workspace_validator
  src/workspace_validator_node.cpp
)

target_include_directories(workspace_validator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(workspace_validator
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_msgs
)

# ── install ────────────────────────────────────────────────────────────────
install(TARGETS workspace_validator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ── tests ─────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
