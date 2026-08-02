# Calibration / lifting support shared by tools and the 3D pipeline.
# Declared before fitra_infer so RTMPose can use the active KeypointFormat
# to validate engine K against the runtime topology.
add_library(fitra_lift STATIC
    geom/frames.cpp
    lift/calib_io.cpp
    lift/triangulator.cpp
    lift/extrinsic_solver.cpp
    lift/apriltag_marker.cpp
    lift/floor_tag_map.cpp
    lift/floor_extrinsic_solver.cpp
    lift/floor_map_sfm.cpp
    lift/charuco_board.cpp
    lift/floor_contact_stabilizer.cpp
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
    opencv_imgproc      # apriltag_marker: cvtColor
    opencv_objdetect    # apriltag_marker: cv::aruco AprilTag detector
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
    camera/v4l2_enumerate.cpp     # setup mode: device enumeration (pure ioctl)
    camera/setup_camera_manager.cpp  # setup mode: JPEG preview
    camera/jpeg_decoder.cpp
    camera/nvjpeg_decoder.cpp     # dlopen loader for libfitra_nvjpeg.so (HW NVJPEG)
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
    ${CMAKE_DL_LIBS}  # dlopen of the isolated HW-NVJPEG .so
)

# --- Isolated Jetson hardware-NVJPEG decoder (libfitra_nvjpeg.so) -------------
# Built only when the Jetson Multimedia API is present. This is a SEPARATE
# shared library, dlopen'd at runtime (NOT linked into the main binary): its
# bundled libjpeg-8b symbols (via libnvjpeg) would otherwise interpose OpenCV's
# libjpeg-turbo and break cv::imdecode. See
# docs/design/core-pipeline-nvjpeg-decode.md.
set(FITRA_MMAPI_DIR "/usr/src/jetson_multimedia_api")
if(EXISTS "${FITRA_MMAPI_DIR}/include/NvJpegDecoder.h")
    set(_mm_classes "${FITRA_MMAPI_DIR}/samples/common/classes")
    # All-GPU front-end (M2) ships a .cu preprocess kernel, so enable the CUDA
    # language here (Jetson-only branch; nvcc is present on the device). The
    # top-level project() stays CXX-only so non-Jetson / no-nvcc builds work.
    enable_language(CUDA)
    add_library(fitra_nvjpeg SHARED
        camera/nvjpeg/fitra_nvjpeg_iso.cpp
        camera/nvjpeg/fitra_nvjpeg_kernels.cu
        "${_mm_classes}/NvJpegDecoder.cpp"
        "${_mm_classes}/NvElement.cpp"
        "${_mm_classes}/NvElementProfiler.cpp"
        "${_mm_classes}/NvLogging.cpp"
        "${_mm_classes}/NvBuffer.cpp"
        "${_mm_classes}/NvBufSurface.cpp"
    )
    set_target_properties(fitra_nvjpeg PROPERTIES
        CXX_STANDARD 17                       # MMAPI sample sources target C++11/14
        CUDA_STANDARD 17
        CUDA_ARCHITECTURES 87                 # Orin (sm_87)
        OUTPUT_NAME fitra_nvjpeg
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"   # land next to `main`
        BUILD_RPATH   "/usr/lib/aarch64-linux-gnu/nvidia;/usr/lib/aarch64-linux-gnu/tegra"
        INSTALL_RPATH "/usr/lib/aarch64-linux-gnu/nvidia;/usr/lib/aarch64-linux-gnu/tegra"
    )
    target_include_directories(fitra_nvjpeg PRIVATE
        "${FITRA_MMAPI_DIR}/include"
        "${FITRA_MMAPI_DIR}/include/libjpeg-8b"
    )
    target_link_directories(fitra_nvjpeg PRIVATE
        /usr/lib/aarch64-linux-gnu/nvidia
        /usr/lib/aarch64-linux-gnu/tegra
    )
    target_link_libraries(fitra_nvjpeg PRIVATE
        nvjpeg nvbufsurface nvbufsurftransform Threads::Threads
        # EGL->CUDA bridge (all-GPU front-end M1): driver API for
        # cuGraphicsEGLRegisterImage, runtime for cudaMemcpy2D.
        CUDA::cudart CUDA::cuda_driver
    )
    # Hide everything but the extern "C" API; silence third-party MMAPI warnings.
    # Gate per-language: nvcc rejects bare -fvisibility, so route it via
    # -Xcompiler for CUDA TUs.
    target_compile_options(fitra_nvjpeg PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fvisibility=hidden -w>
        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-fvisibility=hidden -w>
    )
    set(FITRA_HAVE_NVJPEG ON CACHE INTERNAL "HW NVJPEG decoder available")
    message(STATUS "  HW NVJPEG decoder  = ON (libfitra_nvjpeg.so)")
else()
    message(STATUS "  HW NVJPEG decoder  = OFF (Jetson MMAPI not found)")
endif()

# Pipeline glue: single-camera helper, multi-camera driver, snapshot bus.
add_library(fitra_pipeline STATIC
    pipeline/pose_pipeline.cpp
    pipeline/multi_pipeline.cpp
    pipeline/pose_gate.cpp
    pipeline/snapshot.cpp
    pipeline/calibration_session.cpp
    pipeline/extrinsic_calib_session.cpp
    pipeline/floor_calib_session.cpp
    pipeline/intrinsic_calib_session.cpp
    pipeline/excal_replay_input.cpp
)

target_link_libraries(fitra_pipeline PUBLIC
    fitra_infer
    fitra_camera
    fitra_lift
    opencv_videoio
)

# Shared tracker extraction, smoothing, and WebUI snapshot publication.
add_library(fitra_tracking STATIC
    tracking/tracker_bus.cpp
    tracking/tracker_extract.cpp
    tracking/tracker_extractor.cpp
)

target_include_directories(fitra_tracking PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_tracking PUBLIC
    fitra_lift          # active_keypoint_format() for Halpe26 assert + types
    fitra_pipeline      # Skeleton3DBus::snapshot()
    opencv_core
)

# VMT (Virtual Motion Tracker) integration: OSC 1.0 writer +
# /VMT/Room/Driver wire encoder + world→Driver coordinate transform.
# Targets the SteamVR Driver directly. Shares TrackerRole / kTrackerCount /
# TrackerPose via fitra_tracking headers.
add_library(fitra_vmt STATIC
    vmt/osc_writer.cpp
    vmt/vmt_protocol.cpp
    vmt/vmt_publisher.cpp
    vmt/hmd_pose_receiver.cpp
    vmt/controller_pose_receiver.cpp
    vmt/tracked_pose_receiver.cpp
    vmt/auto_alignment.cpp
    vmt/continuous_aligner.cpp
    vmt/discovery_announce.cpp
    vmt/peer_registry.cpp
    vmt/discovery_beacon.cpp
)

target_include_directories(fitra_vmt PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_vmt PUBLIC
    fitra_tracking
)

# Crow HTTP+WebSocket server.
add_library(fitra_web STATIC
    web/crow_server.cpp
    web/crow_routes_setup.cpp
    web/crow_routes_setup_mode.cpp
)

target_link_libraries(fitra_web PUBLIC
    fitra_config      # setup mode: SetupConfigStore + emit_main_config
    fitra_camera      # setup mode: V4L2 enumeration + preview
    fitra_pipeline
    fitra_tracking
    fitra_vmt         # /stats3d splices VmtPublisher::stats()
    Crow::Crow
)

# Composition root for cpp/build/main: per-mode runners (run / calib-subject /
# calib-extrinsic) and the builders they share. Separate from fitra_pipeline
# because the runners need fitra_vmt/fitra_web, and fitra_vmt already links
# fitra_pipeline via fitra_tracking — the dependency must point this way.
add_library(fitra_app STATIC
    app/camera_builder.cpp
    app/daemon.cpp
    app/excal_live_input.cpp
    app/excal_runner.cpp
    app/floor_calib_runner.cpp
    app/floor_live_input.cpp
    app/idle_evaluator.cpp
    app/intrinsic_calib_runner.cpp
    app/mode_calib_extrinsic.cpp
    app/mode_calib_extrinsic_floor.cpp
    app/mode_calib_intrinsic.cpp
    app/mode_calib_subject.cpp
    app/mode_run.cpp
    app/mode_setup.cpp
    app/output_builder.cpp
    app/paths.cpp
    app/pose_relay_builder.cpp
    app/server_builder.cpp
    app/stats_loop.cpp
    app/threed_builder.cpp
    app/trt_stack.cpp
)

target_include_directories(fitra_app PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(fitra_app PUBLIC
    fitra_config
    fitra_pipeline
    fitra_vmt
    fitra_web
)

# Runtime config loader for cpp/build/main (`--config PATH` YAML).
# Independent of TensorRT/CUDA so its unit test stays light.
add_library(fitra_config STATIC
    config/main_config.cpp
    config/setup_config_store.cpp
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
    fitra_app
    fitra_config
    fitra_pipeline
    fitra_web
    fitra_vmt
)

set_target_properties(fitra_cam_main PROPERTIES
    OUTPUT_NAME "main"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# Ensure the isolated HW-NVJPEG .so is built alongside `main` (dlopen'd at
# runtime; deliberately NOT linked — see the fitra_nvjpeg target above).
if(FITRA_HAVE_NVJPEG)
    add_dependencies(fitra_cam_main fitra_nvjpeg)
endif()
