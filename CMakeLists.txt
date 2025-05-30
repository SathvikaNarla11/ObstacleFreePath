cmake_minimum_required(VERSION 3.10)
project(RRTStarVisualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use vcpkg if installed via environment
set(CMAKE_TOOLCHAIN_FILE "C:/Users/DTIOT0005/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

# Find OpenCV
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Source files
add_executable(RRTGrid src/main.cpp)

# Link OpenCV
target_link_libraries(RRTGrid PRIVATE ${OpenCV_LIBS})

# Optionally show which OpenCV was found
message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")





















