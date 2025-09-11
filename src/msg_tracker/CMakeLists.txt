cmake_minimum_required(VERSION 3.8)
project(msg_tracker)

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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(BABELTRACE2 REQUIRED lttng-ust)
# pkg_check_modules(BABELTRACE2 REQUIRED babeltrace2)

# include_directories(${LTTNG_INCLUDE_DIRS})

add_library(tracker SHARED
  src/tracker.cpp
)
target_include_directories(tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(tracker PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tracker
  rclcpp
  builtin_interfaces
  std_msgs
)

# add_executable(trace_parser src/trace_parser.cpp)
# target_include_directories(trace_parser PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(trace_parser PRIVATE RT_EXEC=${RT_EXEC_VAL})
# target_compile_features(trace_parser PUBLIC c_std_99 cxx_std_17)
# target_link_libraries(trace_parser ${LTTNG_LIBRARIES})
# ament_target_dependencies(
#   trace_parser
#   rclcpp
#   builtin_interfaces
#   std_msgs
# )

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

install(TARGETS tracker
  EXPORT trackerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT trackerTargets
  FILE trackerTargets.cmake
  NAMESPACE msg_tracker::
  DESTINATION lib/cmake/msg_tracker
)

ament_export_targets(trackerTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# install(TARGETS trace_parser
  # DESTINATION lib/${PROJECT_NAME})

ament_package()
