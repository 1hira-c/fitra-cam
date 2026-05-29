# Standalone CLI tools that consume the fitra_infer library.

add_executable(fitra_build_engines
    build_engines.cpp
)

target_link_libraries(fitra_build_engines PRIVATE
    fitra_infer
)

set_target_properties(fitra_build_engines PROPERTIES
    OUTPUT_NAME "build_engines"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# Correctness driver: video -> JSONL of per-frame YOLOX/RTMPose results.
add_executable(fitra_dump_keypoints
    dump_keypoints.cpp
)

target_link_libraries(fitra_dump_keypoints PRIVATE
    fitra_infer
    opencv_videoio
    opencv_imgproc
    opencv_core
)

set_target_properties(fitra_dump_keypoints PROPERTIES
    OUTPUT_NAME "dump_keypoints"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# All-GPU front-end (M2): RTMPose preprocess CUDA kernel correctness vs the CPU
# preprocess_to_blob reference, over a recorded video. dlopen's
# libfitra_nvjpeg.so at runtime (no link dependency); Jetson-only.
if(FITRA_HAVE_NVJPEG)
    add_executable(fitra_gpu_preprocess_check
        gpu_preprocess_check.cpp
    )
    target_link_libraries(fitra_gpu_preprocess_check PRIVATE
        fitra_infer
        opencv_videoio
        opencv_imgproc
        opencv_core
        ${CMAKE_DL_LIBS}
    )
    set_target_properties(fitra_gpu_preprocess_check PROPERTIES
        OUTPUT_NAME "gpu_preprocess_check"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
    )
    add_dependencies(fitra_gpu_preprocess_check fitra_nvjpeg)
endif()

# YOLOX engine per-frame latency benchmark.
add_executable(fitra_det_bench
    det_bench.cpp
)

target_link_libraries(fitra_det_bench PRIVATE
    fitra_infer
    opencv_videoio
    opencv_imgcodecs
    opencv_imgproc
    opencv_core
)

set_target_properties(fitra_det_bench PROPERTIES
    OUTPUT_NAME "det_bench"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# Single-camera live benchmark.
add_executable(fitra_pose_bench
    pose_bench.cpp
)

target_link_libraries(fitra_pose_bench PRIVATE
    fitra_pipeline
    opencv_imgcodecs
    opencv_imgproc
)

set_target_properties(fitra_pose_bench PROPERTIES
    OUTPUT_NAME "pose_bench"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# Calibration YAML validator for measured/ChArUco camera parameters.
add_executable(fitra_check_calibration
    check_calibration.cpp
)

target_link_libraries(fitra_check_calibration PRIVATE
    fitra_lift
)

set_target_properties(fitra_check_calibration PROPERTIES
    OUTPUT_NAME "check_calibration"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

# Recorded two-camera 2D -> 3D dump with reprojection overlays.
add_executable(fitra_dump_keypoints_3d
    dump_keypoints_3d.cpp
)

target_link_libraries(fitra_dump_keypoints_3d PRIVATE
    fitra_infer
    fitra_lift
    opencv_videoio
    opencv_imgproc
    opencv_core
)

set_target_properties(fitra_dump_keypoints_3d PROPERTIES
    OUTPUT_NAME "dump_keypoints_3d"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_executable(fitra_test_triangulator
    test_triangulator.cpp
)

target_link_libraries(fitra_test_triangulator PRIVATE
    fitra_lift
)

set_target_properties(fitra_test_triangulator PROPERTIES
    OUTPUT_NAME "test_triangulator"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_triangulator COMMAND fitra_test_triangulator)

# SlimeVR Firmware UDP wire-format byte-level golden test. Verifies
# handshake / sensor_info / rotation_data / ping_reply layouts and the MAC
# derivation. No SlimeVR server needed.
add_executable(fitra_test_firmware_protocol
    test_firmware_protocol.cpp
)

target_link_libraries(fitra_test_firmware_protocol PRIVATE
    fitra_slimevr
)

set_target_properties(fitra_test_firmware_protocol PROPERTIES
    OUTPUT_NAME "test_firmware_protocol"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_firmware_protocol COMMAND fitra_test_firmware_protocol)

# Halpe26 → 10 SlimeVR tracker extraction test
# (upper arms / chest / waist / upper legs / lower legs / feet).
add_executable(fitra_test_tracker_extract
    test_tracker_extract.cpp
)

target_link_libraries(fitra_test_tracker_extract PRIVATE
    fitra_slimevr
)

set_target_properties(fitra_test_tracker_extract PROPERTIES
    OUTPUT_NAME "test_tracker_extract"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_tracker_extract COMMAND fitra_test_tracker_extract)

# VMT OSC 1.0 wire-format byte-level golden test, including
# the `/VMT/Room/Driver` packet shape.
add_executable(fitra_test_vmt_osc_writer
    test_vmt_osc_writer.cpp
)

target_link_libraries(fitra_test_vmt_osc_writer PRIVATE
    fitra_vmt
)

set_target_properties(fitra_test_vmt_osc_writer PROPERTIES
    OUTPUT_NAME "test_vmt_osc_writer"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_vmt_osc_writer COMMAND fitra_test_vmt_osc_writer)

# world→VMT Driver coordinate transform + TrackerRole index mapping.
add_executable(fitra_test_vmt_protocol
    test_vmt_protocol.cpp
)

target_link_libraries(fitra_test_vmt_protocol PRIVATE
    fitra_vmt
)

set_target_properties(fitra_test_vmt_protocol PROPERTIES
    OUTPUT_NAME "test_vmt_protocol"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_vmt_protocol COMMAND fitra_test_vmt_protocol)

# HMD pose UDP receiver — wire-format parser + bus stale logic.
add_executable(fitra_test_hmd_pose_receiver
    test_hmd_pose_receiver.cpp
)

target_link_libraries(fitra_test_hmd_pose_receiver PRIVATE
    fitra_vmt
)

set_target_properties(fitra_test_hmd_pose_receiver PROPERTIES
    OUTPUT_NAME "test_hmd_pose_receiver"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_hmd_pose_receiver COMMAND fitra_test_hmd_pose_receiver)

# VmtAlignment auto solver — T-pose + 2D Procrustes motion mode.
add_executable(fitra_test_auto_alignment
    test_auto_alignment.cpp
)

target_link_libraries(fitra_test_auto_alignment PRIVATE
    fitra_vmt
)

set_target_properties(fitra_test_auto_alignment PROPERTIES
    OUTPUT_NAME "test_auto_alignment"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_auto_alignment COMMAND fitra_test_auto_alignment)

# Position EMA in TrackerExtractor. Lives in fitra_slimevr because the
# helper is part of tracker_extract.{hpp,cpp}.
add_executable(fitra_test_tracker_extract_pos
    test_tracker_extract_pos.cpp
)

target_link_libraries(fitra_test_tracker_extract_pos PRIVATE
    fitra_slimevr
)

set_target_properties(fitra_test_tracker_extract_pos PROPERTIES
    OUTPUT_NAME "test_tracker_extract_pos"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_tracker_extract_pos COMMAND fitra_test_tracker_extract_pos)

# `--config PATH`: MainOptions YAML loader, CLI overlay precedence,
# and validate_options() — pure parser + checks, no TensorRT / CUDA / OpenCV.
add_executable(fitra_test_main_config
    test_main_config.cpp
)

target_link_libraries(fitra_test_main_config PRIVATE
    fitra_config
)

set_target_properties(fitra_test_main_config PROPERTIES
    OUTPUT_NAME "test_main_config"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)

add_test(NAME test_main_config COMMAND fitra_test_main_config)
