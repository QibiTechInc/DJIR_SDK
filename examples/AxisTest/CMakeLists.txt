cmake_minimum_required(VERSION 3.13)

###############################################################################
## EXECUTABLE-PROJECT
## Rotates yaw, roll and pitch one axis at a time.
###############################################################################
project(DJIR_AxisTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(${PROJECT_NAME} DJIR_SDK)
