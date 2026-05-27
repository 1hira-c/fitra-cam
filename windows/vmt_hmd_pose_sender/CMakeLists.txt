cmake_minimum_required(VERSION 3.20)
project(vmt_hmd_pose_sender CXX)

#
# Phase 15 M1: SteamVR overlay app that reads HMD pose via OpenVR and
# publishes it to the Jetson over OSC/UDP.
#
# This subtree is intentionally NOT added from the top-level CMakeLists.txt
# in `cpp/` — that file targets Jetson Linux only. Configure this directory
# directly from a Windows host:
#
#   cmake -S windows/vmt_hmd_pose_sender -B build_win \
#         -G "Visual Studio 17 2022" -A x64 \
#         -DOPENVR_DIR=<path>/openvr-1.23.7 \
#         -DOSCPACK_DIR=<path>/oscpack_1_1_0
#   cmake --build build_win --config Release
#
# Both OPENVR_DIR and OSCPACK_DIR default to the in-repo `refs/...` paths
# under VirtualMotionTracker, so a developer who clones VMT next to
# fitra-cam can configure without flags. See README.md for details.
#

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT WIN32)
    message(FATAL_ERROR
        "vmt_hmd_pose_sender targets Windows (MSVC) only. "
        "It must be configured on a Windows host that runs SteamVR.")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- OpenVR -----------------------------------------------------------------
set(OPENVR_DIR "" CACHE PATH
    "Path to openvr-1.23.7 source tree (headers/, lib/win64/, bin/win64/).")
find_package(openvr REQUIRED)

# --- oscpack ---------------------------------------------------------------
set(OSCPACK_DIR "" CACHE PATH
    "Path to oscpack_1_1_0 source tree (with ip/ and osc/ subdirectories).")

if(NOT OSCPACK_DIR)
    # Default to VMT's bundled copy if the user cloned VMT next to fitra-cam.
    set(_oscpack_guess "${CMAKE_CURRENT_SOURCE_DIR}/../../../refs/VirtualMotionTracker/oscpack_1_1_0")
    if(EXISTS "${_oscpack_guess}/CMakeLists.txt")
        set(OSCPACK_DIR "${_oscpack_guess}")
    else()
        message(FATAL_ERROR
            "OSCPACK_DIR not set. Pass -DOSCPACK_DIR=<path-to-oscpack_1_1_0>.")
    endif()
endif()

if(NOT EXISTS "${OSCPACK_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "OSCPACK_DIR=${OSCPACK_DIR} does not contain CMakeLists.txt.")
endif()

# Build oscpack as a static library from its in-tree CMakeLists.
add_subdirectory("${OSCPACK_DIR}"
                 "${CMAKE_CURRENT_BINARY_DIR}/oscpack" EXCLUDE_FROM_ALL)
target_include_directories(oscpack PUBLIC "${OSCPACK_DIR}")

# --- vmt_hmd_pose_sender ----------------------------------------------------
add_executable(vmt_hmd_pose_sender
    src/main.cpp
    src/osc_send.cpp
)

target_include_directories(vmt_hmd_pose_sender PRIVATE
    src
)

target_link_libraries(vmt_hmd_pose_sender PRIVATE
    openvr::openvr
    oscpack
    ws2_32
    winmm
)

# Copy openvr_api.dll next to the executable so it can be run from
# `Release/` without manually placing the DLL.
if(TARGET openvr::openvr)
    get_target_property(_openvr_dll openvr::openvr OPENVR_DLL_PATH)
    if(_openvr_dll AND EXISTS "${_openvr_dll}")
        add_custom_command(TARGET vmt_hmd_pose_sender POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_openvr_dll}"
                "$<TARGET_FILE_DIR:vmt_hmd_pose_sender>"
            COMMENT "Copying openvr_api.dll next to vmt_hmd_pose_sender.exe")
    endif()
endif()

target_compile_definitions(vmt_hmd_pose_sender PRIVATE
    _WIN32_WINNT=0x0A00     # Win10+
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)

if(MSVC)
    target_compile_options(vmt_hmd_pose_sender PRIVATE /W4 /permissive-)
endif()
