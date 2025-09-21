# Stage 1: Build environment
# Use a base image with CUDA and a common Ubuntu distribution.
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 AS build

# Set the working directory inside the container
WORKDIR /app

# Install system dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git \
    g++ \
    python3-pip \
    libc++-dev \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev \
    libtbb-dev \
    freeglut3-dev \
    libopenni2-dev \
    liboctomap-dev \
    nvidia-cuda-dev \
    make \
    wget \
    unzip && \
    pip install 'cmake>=3.24' && \
    rm -rf /var/lib/apt/lists/*

# Clone the repository and its submodules
RUN git clone --recursive https://github.com/goldoak/GSFusion.git
WORKDIR /app/GSFusion

# Set up LibTorch
RUN wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cu118.zip -d third_party/ && \
    rm libtorch-cxx11-abi-shared-with-deps-2.0.1+cu118.zip

# Set up Open3D
RUN wget https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz && \
    tar -xvf open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz -C third_party && \
    mv third_party/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0 third_party/open3d && \
    rm open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz

# Build the project in release mode
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && \
    cmake --build build -- -j

# --- End of Build Stage ---

# Stage 2: Final image
# Use a lightweight runtime image
FROM nvidia/cuda:11.7.1-runtime-ubuntu22.04

# Copy repository lists and keys from the build stage.
COPY --from=build /etc/apt/sources.list.d/ /etc/apt/sources.list.d/
COPY --from=build /usr/share/keyrings/ /usr/share/keyrings/
COPY --from=build /etc/apt/trusted.gpg.d/ /etc/apt/trusted.gpg.d/

# Install the required RUNTIME libraries.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libtbb2 \
    libopenni2-0 \
    libc++1 \
    libopencv-core4.5d \
    libopencv-imgproc4.5d \
    libopencv-highgui4.5d \
    libopencv-flann4.5d \
    libgl1 \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app/GSFusion

# Copy the entire 'third_party', 'config' and 'parameter' directories
COPY --from=build /app/GSFusion/third_party ./third_party
COPY --from=build /app/GSFusion/config ./config
COPY --from=build /app/GSFusion/parameter ./parameter/

# Create the build path and copy the executable
RUN mkdir -p ./build/app
COPY --from=build /app/GSFusion/build/app/gsfusion ./build/app/

# Set the library path for the copied .so files
ENV LD_LIBRARY_PATH=/app/GSFusion/third_party/libtorch/lib:/app/GSFusion/third_party/open3d/lib:${LD_LIBRARY_PATH}
