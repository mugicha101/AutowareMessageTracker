cmake_minimum_required(VERSION 3.8)
project(node_latency)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# toggle for using realtime executor (EDF) or default ros2 executors (rclcpp::executors::SingleThreadedExecutor, rclcpp::executors::MultiThreadedExecutor)
if(RT_EXEC)
  set(RT_EXEC_VAL 1)
else()
  set(RT_EXEC_VAL 0)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msg_tracker REQUIRED)
find_package(node_latency_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(main src/main.cpp)
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(main PRIVATE RT_EXEC=${RT_EXEC_VAL})
target_compile_features(main PUBLIC c_std_99 cxx_std_17)
target_link_libraries(main msg_tracker::tracker)
ament_target_dependencies(
  main
  rclcpp
  msg_tracker
  std_msgs
  node_latency_interfaces
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  # ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
