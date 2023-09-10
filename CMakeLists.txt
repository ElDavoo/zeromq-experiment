cmake_minimum_required(VERSION 3.16.0)

project(zeromq-experiment VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)

# Set build flags here
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-function")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
add_subdirectory(common)

add_subdirectory(zeromq)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

