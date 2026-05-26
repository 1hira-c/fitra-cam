# Find module for the openvr-1.23.7 source distribution shipped with VMT.
#
# Inputs (cache):
#   OPENVR_DIR — path to the openvr-1.23.7 tree (with headers/, lib/win64/,
#                bin/win64/). If empty, this module guesses
#                `<source>/../../../refs/VirtualMotionTracker/openvr-1.23.7`
#                so developers who clone VMT next to fitra-cam can configure
#                without passing -DOPENVR_DIR.
#
# Output target:
#   openvr::openvr — IMPORTED STATIC-equivalent (uses the prebuilt
#                    openvr_api.lib import library + headers). Also carries
#                    the property OPENVR_DLL_PATH so consumers can copy the
#                    DLL next to their executable.

if(NOT OPENVR_DIR)
    # CMAKE_CURRENT_SOURCE_DIR here is the directory of the calling
    # CMakeLists.txt (windows/vmt_hmd_pose_sender/), three levels above
    # /home/<user>/Documents/.
    set(_openvr_guess "${CMAKE_CURRENT_SOURCE_DIR}/../../../refs/VirtualMotionTracker/openvr-1.23.7")
    if(EXISTS "${_openvr_guess}/headers/openvr.h")
        set(OPENVR_DIR "${_openvr_guess}" CACHE PATH "openvr-1.23.7 root" FORCE)
    endif()
endif()

find_path(OPENVR_INCLUDE_DIR
    NAMES openvr.h
    HINTS "${OPENVR_DIR}/headers"
)

find_library(OPENVR_LIBRARY
    NAMES openvr_api
    HINTS "${OPENVR_DIR}/lib/win64"
)

find_file(OPENVR_DLL
    NAMES openvr_api.dll
    HINTS "${OPENVR_DIR}/bin/win64"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(openvr
    REQUIRED_VARS OPENVR_INCLUDE_DIR OPENVR_LIBRARY OPENVR_DLL
)

if(openvr_FOUND AND NOT TARGET openvr::openvr)
    add_library(openvr::openvr UNKNOWN IMPORTED)
    set_target_properties(openvr::openvr PROPERTIES
        IMPORTED_LOCATION "${OPENVR_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OPENVR_INCLUDE_DIR}"
        OPENVR_DLL_PATH "${OPENVR_DLL}"
    )
endif()

mark_as_advanced(OPENVR_INCLUDE_DIR OPENVR_LIBRARY OPENVR_DLL)
