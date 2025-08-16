cmake_minimum_required(VERSION 3.20)
project(MotionPlanning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Configuring Debug build")
    set(CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O0")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Configuring Release build")
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
endif()

find_package(Protobuf REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "Protobuf_VERSION: ${Protobuf_VERSION}")
message(STATUS "Protobuf_PROTOC_EXECUTABLE: ${Protobuf_PROTOC_EXECUTABLE}")
message(STATUS "Protobuf_INCLUDE_DIR: ${Protobuf_INCLUDE_DIR}")
message(STATUS "Protobuf_LIBRARIES: ${Protobuf_LIBRARIES}")

set(Protobuf_PROTOC_EXECUTABLE ${Protobuf_PROTOC_EXECUTABLE} CACHE FILEPATH "Protobuf compiler" FORCE)

include_directories("/usr/include/eigen3")
include_directories(${Protobuf_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/foxglove/include)

# Find all Foxglove SDK source files
set(foxglove_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/foxglove")
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
        "${foxglove_SOURCE_DIR}/src/*.cpp"
        "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

set(PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/protos/planning_info.proto)

# Create a target for the protobuf generation
add_library(planning_proto STATIC)
protobuf_generate(
    TARGET planning_proto
    LANGUAGE cpp
    PROTOS ${PROTO_FILES}
)
target_include_directories(planning_proto PUBLIC
    ${Protobuf_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)
# Link Protocol Buffers to fruit_proto
target_link_libraries(planning_proto PUBLIC ${Protobuf_LIBRARIES})

file(GLOB SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")
set(3RDPARTY spdlog::spdlog yaml-cpp::yaml-cpp ${foxglove_SOURCE_DIR}/lib/libfoxglove.a ${Protobuf_LIBRARIES} pthread dl)

add_executable(motion_planning ${SOURCE_FILES})
target_sources(motion_planning PRIVATE ${FOXGLOVE_SOURCES})
target_link_libraries(motion_planning planning_proto ${3RDPARTY})
