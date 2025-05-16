cmake_minimum_required(VERSION 3.8)
project(tagtransform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)

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

# add_executable(tagtransform_node src/tracking_subscriber.cpp)
# ament_target_dependencies(tagtransform_node
#   rclcpp
#   geometry_msgs
#   tf2_ros
#   px4_msgs
# )

# install(TARGETS
#   tagtransform_node
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(tagtransform_node src/tracking_subscriber.cpp)
target_include_directories(tagtransform_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(tagtransform_node
  rclcpp geometry_msgs tf2_ros px4_msgs
)

install(TARGETS
  tagtransform_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
