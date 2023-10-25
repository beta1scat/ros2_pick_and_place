cmake_minimum_required(VERSION 3.5)
project(tf_pick)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
  moveit_servo
  sensor_msgs
  std_srvs
)

# add_executable(test_node src/test_node.cpp)
# target_include_directories(test_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# install(TARGETS test_node
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(example_gripper src/example_gripper.cpp)
ament_target_dependencies(example_gripper
  rclcpp
  std_msgs
)
install(TARGETS example_gripper DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY json DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})


# add_executable(motion_planning src/motion_planning.cpp)
# target_include_directories(motion_planning PUBLIC include)
# ament_target_dependencies(motion_planning ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# install(TARGETS motion_planning DESTINATION lib/${PROJECT_NAME})

# add_executable(move_group src/move_group.cpp)
# target_include_directories(move_group PUBLIC include)
# ament_target_dependencies(move_group ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# install(TARGETS move_group DESTINATION lib/${PROJECT_NAME})

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Convert native path into a cmake-style path with '/'
file(TO_CMAKE_PATH "$ENV{HALCONEXAMPLES}" HALCONEXAMPLES)
list(APPEND CMAKE_MODULE_PATH "${HALCONEXAMPLES}/cmake")
include(UseHALCON)
add_executable(sfm src/sfm.cpp)
target_link_libraries( sfm HALCON::CppInt )
ament_target_dependencies(sfm ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS sfm DESTINATION lib/${PROJECT_NAME})

find_package(ros2_data REQUIRED)

add_executable(plan_and_grasp src/plan_and_grasp.cpp)
ament_target_dependencies(plan_and_grasp ros2_data ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS plan_and_grasp DESTINATION lib/${PROJECT_NAME})

add_executable(robot_pose_print src/robot_pose_print.cpp)
ament_target_dependencies(robot_pose_print ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS robot_pose_print DESTINATION lib/${PROJECT_NAME})


# find_package(Glog REQUIRED)
# include_directories(${GLOG_INCLUDE_DIR})

# # find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(PCL 1.11 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})


# find_package(OpenCV 4 REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

# include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/conan.cmake)
# conan_cmake_run(CONANFILE conan-dependencies.txt
#                 BASIC_SETUP CMAKE_TARGETS
#                 BUILD_TYPE "Release")

# add_executable(test_surface_matching_cpu src/test_surface_matching_cpu.cpp)
# target_include_directories(test_surface_matching_cpu PUBLIC include)
# target_link_libraries(
#     test_surface_matching_cpu
#     CONAN_PKG::surface_matching_cpu
#     CONAN_PKG::point_cloud_utils
#     ${GLOG_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES})
# ament_target_dependencies(test_surface_matching_cpu ${THIS_PACKAGE_INCLUDE_DEPENDS})
# install(TARGETS test_surface_matching_cpu DESTINATION lib/${PROJECT_NAME})


# add_executable(point_cloud src/point_cloud.cpp)
# target_link_directories(point_cloud PUBLIC lib)
# target_link_libraries(
#   point_cloud
#   TFPickPoint
#   CONAN_PKG::common
#   CONAN_PKG::surface_matching_cpu
#   CONAN_PKG::point_cloud_utils
#   ${GLOG_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES})
# target_include_directories(point_cloud PUBLIC include)
# ament_target_dependencies(point_cloud ${THIS_PACKAGE_INCLUDE_DEPENDS})
# install(TARGETS point_cloud DESTINATION lib/${PROJECT_NAME})

# add_executable(moveit_cpp src/moveit_cpp.cpp)
# target_include_directories(moveit_cpp PUBLIC include)
# target_link_directories(moveit_cpp PUBLIC lib)
# target_link_libraries(
#   moveit_cpp
#   TFPickPoint
#   CONAN_PKG::common
#   CONAN_PKG::surface_matching_cpu
#   CONAN_PKG::point_cloud_utils
#   ${GLOG_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES})
# ament_target_dependencies(moveit_cpp  ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# install(TARGETS moveit_cpp DESTINATION lib/${PROJECT_NAME})

# add_executable(vacuum_gripper src/vacuum_gripper.cpp)
# ament_target_dependencies(vacuum_gripper  ${THIS_PACKAGE_INCLUDE_DEPENDS})
# install(TARGETS vacuum_gripper DESTINATION lib/${PROJECT_NAME})

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(pick_point src/pick_point.cpp)
# target_link_directories(pick_point PUBLIC lib)
# target_include_directories(pick_point PUBLIC include)
# target_link_libraries(pick_point
#                       TFPickPoint
#                       CONAN_PKG::common
#                      ${PCL_LIBRARIES})
# ament_target_dependencies(pick_point ${THIS_PACKAGE_INCLUDE_DEPENDS})
# install(TARGETS pick_point DESTINATION lib/${PROJECT_NAME})


ament_package()

