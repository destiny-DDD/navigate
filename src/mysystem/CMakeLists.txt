cmake_minimum_required(VERSION 3.8)
project(mysystem)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(odom_to_tf_core src/odom_to_tf_core.cpp)
target_include_directories(odom_to_tf_core PUBLIC include)
ament_target_dependencies(odom_to_tf_core geometry_msgs nav_msgs)

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf odom_to_tf_core)
ament_target_dependencies(odom_to_tf rclcpp tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_sim_world_assets test/test_sim_world_assets.py)
  ament_add_gtest(test_odom_to_tf test/test_odom_to_tf.cpp)
  target_link_libraries(test_odom_to_tf odom_to_tf_core)
endif()

install(TARGETS
  odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config urdf meshes launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
