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

# Phase 1 correctness driver: video -> JSONL of per-frame YOLOX/RTMPose results.
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

# Phase 2: single-camera live benchmark.
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

# Phase 7: recorded two-camera 2D -> 3D dump with reprojection overlays.
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

# Phase 11: SlimeVR Firmware UDP wire-format byte-level golden test. Verifies
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

# Phase 11: Halpe26 → SlimeVR tracker extraction test. M3 rewrites the
# layout to 10 trackers (upper arms / chest / waist / upper legs / lower
# legs / feet); the test currently targets the legacy 8-tracker VMC layout
# until that milestone lands.
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

# Backlog (`--config PATH`): MainOptions YAML loader, CLI overlay precedence,
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
