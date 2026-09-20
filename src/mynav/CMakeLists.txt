cmake_minimum_required(VERSION 3.8)
project(mynav)

find_package(ament_cmake REQUIRED)

# 本包无编译目标，只把 launch 与 config 装进 share/，供 ros2 launch 读取。
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查与 cpplint：本包是纯配置包，且不在 git 仓库的常规位置。
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
