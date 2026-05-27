# Calibration / lifting support shared by tools and the 3D pipeline.
# Declared before fitra_infer so RTMPose can use the active KeypointFormat
# to validate engine K against the runtime topology.
add_library(fitra_lift STATIC
    lift/calib_io.cpp
    lift/triangulator.cpp
    lift/kalman.cpp
    lift/ik.cpp
    lift/subject_profile.cpp
    lift/pose_recognizer.cpp
    lift/keypoint_format.cpp
)

target_include_directories(fitra_lift PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_lift PUBLIC
    opencv_core
    opencv_calib3d
)

# Inference library (TRT engine wrapper, ONNX -> engine builder, YOLOX, RTMPose).
add_library(fitra_infer STATIC
    infer/trt_engine.cpp
    infer/trt_builder.cpp
    infer/yolox.cpp
    infer/rtmpose.cpp
)

target_include_directories(fitra_infer PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_infer PUBLIC
    CUDA::cudart
    TensorRT::TensorRT
    Threads::Threads
    opencv_core
    opencv_imgproc
    fitra_lift          # RTMPose validates engine K against the active topology
)

# Camera capture + JPEG decode + per-camera decode worker
# (FrameSource parallelizes CPU decode across cameras).
add_library(fitra_camera STATIC
    camera/v4l2_capture.cpp
    camera/jpeg_decoder.cpp
    camera/frame_source.cpp
)

target_include_directories(fitra_camera PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_camera PUBLIC
    fitra_infer       # FrameSource owns a Yolox (per-camera context)
    Threads::Threads
    opencv_core
    opencv_imgcodecs
)

# Pipeline glue: single-camera helper, multi-camera driver, snapshot bus.
add_library(fitra_pipeline STATIC
    pipeline/pose_pipeline.cpp
    pipeline/multi_pipeline.cpp
    pipeline/snapshot.cpp
    pipeline/calibration_session.cpp
)

target_link_libraries(fitra_pipeline PUBLIC
    fitra_infer
    fitra_camera
    fitra_lift
    opencv_videoio
)

# SlimeVR integration: tracker extraction (rotation synthesis) +
# native Firmware UDP publisher (port 6969, 10-tracker layout).
add_library(fitra_slimevr STATIC
    slimevr/firmware_protocol.cpp
    slimevr/native_publisher.cpp
    slimevr/slime_tracker_bus.cpp
    slimevr/tracker_extract.cpp
    slimevr/tracker_extractor.cpp
)

target_include_directories(fitra_slimevr PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_slimevr PUBLIC
    fitra_lift          # active_keypoint_format() for Halpe26 assert + types
    fitra_pipeline      # Skeleton3DBus::snapshot()
    opencv_core
)

# VMT (Virtual Motion Tracker) integration: OSC 1.0 writer +
# /VMT/Room/Driver wire encoder + world→Driver coordinate transform.
# Bypasses SlimeVR Server (targets SteamVR Driver directly). Shares
# TrackerRole / kTrackerCount / SlimeTracker via fitra_slimevr headers.
add_library(fitra_vmt STATIC
    vmt/osc_writer.cpp
    vmt/vmt_protocol.cpp
    vmt/vmt_publisher.cpp
    vmt/hmd_pose_receiver.cpp
    vmt/auto_alignment.cpp
)

target_include_directories(fitra_vmt PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_vmt PUBLIC
    fitra_slimevr
)

# Crow HTTP+WebSocket server.
add_library(fitra_web STATIC
    web/crow_server.cpp
)

target_link_libraries(fitra_web PUBLIC
    fitra_pipeline
    fitra_slimevr     # /stats3d reads NativePublisher::stats() when attached
    fitra_vmt         # /stats3d also splices VmtPublisher::stats()
    Crow::Crow
)

# Runtime config loader for cpp/build/main (`--config PATH` YAML).
# Independent of TensorRT/CUDA so its unit test stays light.
add_library(fitra_config STATIC
    config/main_config.cpp
)

target_include_directories(fitra_config PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_config PUBLIC
    yaml-cpp::yaml-cpp
)

# Top-level driver: N cameras -> shared TRT -> Crow web.
add_executable(fitra_cam_main
    main.cpp
)

target_link_libraries(fitra_cam_main PRIVATE
    fitra_config
    fitra_pipeline
    fitra_web
    fitra_vmt
)

set_target_properties(fitra_cam_main PROPERTIES
    OUTPUT_NAME "main"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
