cmake_minimum_required(VERSION 3.16)
project(djir_ros2_driver LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# DJIR_SDK lives two levels up in this repository (resolve symlinks so the
# package can be symlinked into a colcon workspace).
get_filename_component(_pkg_dir "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(_sdk_default "${_pkg_dir}/../.." ABSOLUTE)
set(DJIR_SDK_ROOT "${_sdk_default}" CACHE PATH "Path to the DJIR_SDK repository")
add_subdirectory(${DJIR_SDK_ROOT}/3rdparty/custom_crc ${CMAKE_CURRENT_BINARY_DIR}/custom_crc)
add_subdirectory(${DJIR_SDK_ROOT}/src ${CMAKE_CURRENT_BINARY_DIR}/djir_sdk)

add_executable(djir_driver_node src/djir_driver_node.cpp)
target_compile_features(djir_driver_node PRIVATE cxx_std_17)
target_link_libraries(djir_driver_node
  DJIR_SDK
  rclcpp::rclcpp
  diagnostic_updater::diagnostic_updater
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

install(TARGETS djir_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
