cmake_minimum_required(VERSION 3.8)
project(simulator)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# library
# TODO refactor to library
# add_library(${PROJECT_NAME} STATIC
#     src/2dj_simulator.cpp
#   )

# fine dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(2dj_simulator demo/2dj_simulator.cpp)
ament_target_dependencies(2dj_simulator rclcpp Eigen3 nav_msgs geometry_msgs rclcpp tf2_ros)
target_include_directories(2dj_simulator PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/inc
  )
# TODO refactor to library
#target_link_libraries(2dj_simulator Eigen3::Eigen)

install(TARGETS
  2dj_simulator
  DESTINATION lib/${PROJECT_NAME}
)
