cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

add_library(matplotlib_cpp INTERFACE)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)

find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE Python3::NumPy)
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)

file(GLOB SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")
set(3RDPARTY matplotlib_cpp fmt::fmt spdlog::spdlog yaml-cpp::yaml-cpp)

add_executable(motion_planning ${SOURCE_FILES})
target_link_libraries(motion_planning ${3RDPARTY})
