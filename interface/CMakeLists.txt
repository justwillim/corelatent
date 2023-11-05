cmake_minimum_required(VERSION 3.8)
project(interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)

# message generation
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/Polynomial.msg"
 )

# library
add_library(${PROJECT_NAME} STATIC src/polynomial_trajectory.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
  $<INSTALL_INTERFACE:include>
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(inc)

install(
  DIRECTORY inc/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# executables
add_executable(polynomial_talker demo/polynomial_msg_demo.cpp)
ament_target_dependencies(polynomial_talker rclcpp)
target_link_libraries(polynomial_talker ${PROJECT_NAME})
rosidl_target_interfaces(polynomial_talker ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

add_executable(path_sampling_vis demo/rviz2_vis_p3t.cpp)
ament_target_dependencies(path_sampling_vis rclcpp nav_msgs)
target_link_libraries(path_sampling_vis ${PROJECT_NAME})
rosidl_target_interfaces(path_sampling_vis ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
target_include_directories(path_sampling_vis PUBLIC
${EIGEN3_INCLUDE_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/inc
)

install(TARGETS
  polynomial_talker
  path_sampling_vis
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(polynomial_3d_trajectory_test test/polynomial_trajectory_test.cpp)
  target_link_libraries(polynomial_3d_trajectory_test ${PROJECT_NAME})
  target_include_directories(polynomial_3d_trajectory_test PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
  )
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

# Exporting headers
install(
  DIRECTORY inc/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(inc)

ament_package()
