# fitra-cam C++ runtime image (Jetson Orin Nano Super, JetPack 6.2.x).
#
# Base image bundles CUDA 12.6 + cuDNN + TensorRT 10.3 + OpenCV-less L4T userspace.
# Host L4T R36.5.0 / TRT 10.3.0.30 stays binary-compatible with r36.4.x containers
# (same TRT major.minor), so engines built inside the container can be reused
# from the host bind-mounted ./outputs/tensorrt_engines/ between runs.
#
# Build context = repo root. .dockerignore keeps cpp/build, outputs, python/.venv
# out of the layer.

FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config ca-certificates \
        libopencv-dev libv4l-dev v4l-utils \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Layer A: only the CMake config and FetchContent-relevant files first, so
# subsequent source-only edits don't bust the asio/Crow fetch cache.
COPY cpp/CMakeLists.txt cpp/CMakeLists.txt
COPY cpp/cmake          cpp/cmake
COPY cpp/src/CMakeLists.txt   cpp/src/CMakeLists.txt
COPY cpp/tools/CMakeLists.txt cpp/tools/CMakeLists.txt

# Jetson L4T quirk: libnvinfer.so has NEEDED entries for libnvdla_compiler.so /
# libnvcudla.so which only get bind-mounted at *runtime* by nvidia-container-runtime.
# At `docker build` time those host driver libs are absent, so the linker fails with
# "undefined reference to nvdla::...". Telling ld to defer unresolved symbols from
# shared libs to runtime resolves cleanly — the symbols are present when the
# container runs under `runtime: nvidia`.
ENV LDFLAGS="-Wl,--allow-shlib-undefined"

# Pre-populate FetchContent (asio + Crow) and configure once.
RUN cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXE_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
        -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
    || true

# Layer B: now bring the rest of the C++ sources and the web frontend.
COPY cpp /workspace/cpp
COPY web /workspace/web

# Reconfigure + build. Reconfigure is cheap once _deps/ is populated.
RUN cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXE_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
        -DCMAKE_SHARED_LINKER_FLAGS="-Wl,--allow-shlib-undefined" \
 && cmake --build cpp/build -j"$(nproc)"

EXPOSE 8000

# Default to --help so a bare `docker run fitra-cam` is non-destructive and
# tells the operator which flags are expected.
ENTRYPOINT ["/workspace/cpp/build/main"]
CMD ["--help"]
