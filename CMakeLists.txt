cmake_minimum_required(VERSION 3.8)
project(create_demo_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS 
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  ur_msgs
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

add_executable(human_to_robot_handover src/human_to_robot_handover_v2.cpp)
ament_target_dependencies(human_to_robot_handover ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
human_to_robot_handover
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
