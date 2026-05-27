cmake_minimum_required(VERSION 3.22)

# We don't ship .cu kernels — libcudart + TensorRT is enough, and
# find_package(CUDAToolkit) below pulls those in without requiring nvcc on PATH.
project(fitra_cam CXX)

# -------- compiler --------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

# -------- module path --------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# -------- deps --------
# Jetson Orin Nano Super: CUDA 12.6 / TensorRT 10.3 (apt-installed).
find_package(CUDAToolkit REQUIRED)
find_package(TensorRT REQUIRED)

# OpenCV (apt: libopencv-dev). The correctness/dump tools need
# imgproc + imgcodecs + videoio; the hot pipeline avoids OpenCV.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d)

# Threads
find_package(Threads REQUIRED)

# Crow (header-only HTTP/WebSocket) requires either standalone asio or
# Boost.Asio. apt's libboost-dev on JetPack ships headers only (no
# boost_system CMake config), so we fetch standalone asio headers and
# expose ASIO_INCLUDE_DIR before Crow's find_package(asio).
include(FetchContent)
FetchContent_Declare(
    asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG        asio-1-28-2
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(asio)
set(ASIO_INCLUDE_DIR "${asio_SOURCE_DIR}/asio/include"
    CACHE PATH "asio header dir (consumed by Crow's Findasio)" FORCE)

set(CROW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CROW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(CROW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    Crow
    GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
    GIT_TAG        v1.2.1.2
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Crow)

# yaml-cpp — used by fitra_config to load main.cpp runtime YAML
# (`--config PATH`). yaml-cpp ships a CMake target `yaml-cpp::yaml-cpp` since
# 0.8.0; we disable its tests/tools/install side-effects to keep the FetchContent
# subdirectory quiet.
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL       OFF CACHE BOOL "" FORCE)
set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(yaml-cpp)

# -------- targets --------
add_subdirectory(src)
add_subdirectory(tools)

# -------- summary --------
message(STATUS "")
message(STATUS "fitra_cam build configuration")
message(STATUS "  CMAKE_BUILD_TYPE   = ${CMAKE_BUILD_TYPE}")
message(STATUS "  CXX standard       = ${CMAKE_CXX_STANDARD}")
message(STATUS "  CUDA toolkit       = ${CUDAToolkit_VERSION} (${CUDAToolkit_LIBRARY_ROOT})")
message(STATUS "  TensorRT version   = ${TensorRT_VERSION}")
message(STATUS "  TensorRT include   = ${TensorRT_INCLUDE_DIRS}")
message(STATUS "  TensorRT libraries = ${TensorRT_LIBRARIES}")
message(STATUS "  OpenCV version     = ${OpenCV_VERSION}")
message(STATUS "  Crow source dir    = ${crow_SOURCE_DIR}")
message(STATUS "")
