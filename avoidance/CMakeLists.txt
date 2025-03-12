cmake_minimum_required(VERSION 3.8)
project(avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_safety_lib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(
    ${PROJECT_NAME}_node
    src/avoidance_node.cpp
    src/Avoidance.cpp
    include/Avoidance.hpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    visualization_msgs
    px4_safety_lib
    tf2_geometry_msgs
    nav_msgs
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()