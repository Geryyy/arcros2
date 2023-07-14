cmake_minimum_required(VERSION 3.8)
project(arcros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ament_python_install_package(${PROJECT_NAME})


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


set(msg_files
  "msg/RobotState.msg"
  "msg/PoseTraj.msg"
  "msg/TaskSpaceTraj.msg"
  "msg/Pose.msg"
  "msg/PoseState.msg"
  "msg/Joints.msg"
  "msg/JointSpaceTraj.msg"
  "msg/ToolParam.msg"
  "msg/CartVelTraj.msg"

)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# include_directories(include)
# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# install(TARGETS
#   talker
#   DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS
  scripts/iiwajoco.py
  scripts/IiwaArcNode.py
  rc/arcros2/Iiwa.py
  # src/arcros2/LinearAxis.py
  # src/arcros2/__init__.py
  # src/arcros2/utils.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install all the necessary asset files
install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
  FILES_MATCHING PATTERN "*.xml" PATTERN "*.stl"
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
