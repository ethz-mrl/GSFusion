# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.24)
project(GSFusion LANGUAGES C CXX CUDA)

option(SE_OPENMP "Compile supereight with OpenMP" ON)

# Define the absolute path to LibTorch
get_filename_component(PROJ_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(LIBTORCH_DIR "${PROJ_ROOT_DIR}/third_party/libtorch")
list(APPEND CMAKE_PREFIX_PATH ${LIBTORCH_DIR})

# Core count calculation
include(ProcessorCount)
ProcessorCount(total_cores)
if (NOT DEFINED total_cores OR total_cores EQUAL 0)
    set(total_cores 1)
    set(used_cores 1)
elseif (total_cores GREATER 1)
    math(EXPR used_cores "${total_cores} - 2") # use total_cores-2 if total_cores > 1
endif ()
set(ENV{MAKEFLAGS} "-j${used_cores}")
message(STATUS "Building with ${used_cores} out of ${total_cores} available cores")

# Find dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Boost REQUIRED) # We only need Boost.Pool which is a header-only library.
find_package(octomap REQUIRED)
find_package(TBB COMPONENTS tbb)
if(TBB_tbb_FOUND)
    set(SE_TBB 1)
endif()

# Torch setup
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
find_package(Torch REQUIRED)

# Check CUDA version
find_package(CUDAToolkit REQUIRED)
if (CUDAToolkit_VERSION VERSION_LESS "11.7")
    message(FATAL_ERROR "This project requires CUDA 11.7 or higher")
endif ()

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Enable interprocedural optimization (IPO/LTO) for Release if supported
include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED)
if(IPO_SUPPORTED)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
endif()

# Compile the dependencies
add_subdirectory(third_party)

# Compile the gaussian splatting library
set(GS_LIB_NAME "gsmodel")
add_library(${GS_LIB_NAME} STATIC
    "src/gs/quad_tree.cu"
    "src/gs/gaussian.cu"
    "src/gs/rasterizer.cu"
    "src/gs/rasterize_points.cu"
    "cuda_rasterizer/src/backward.cu"
    "cuda_rasterizer/src/forward.cu"
    "cuda_rasterizer/src/rasterizer_impl.cu")
target_include_directories(${GS_LIB_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include 
    ${PROJECT_SOURCE_DIR}/cuda_rasterizer/include 
    ${PROJECT_SOURCE_DIR}/third_party/tinyply/source)
set_target_properties(${GS_LIB_NAME} PROPERTIES
    CUDA_ARCHITECTURES native
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
target_compile_options(${GS_LIB_NAME} PRIVATE -O3 -use_fast_math -Xcompiler -Ofast )
find_library(NVML_LIBRARY nvidia-ml PATHS /usr/lib/x86_64-linux-gnu)
target_link_libraries(${GS_LIB_NAME} PUBLIC
    ${TORCH_LIBRARIES}
    CUDA::cudart
    CUDA::curand
    CUDA::cublas
    ${NVML_LIBRARY}
    tinyply
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    glm
    ${OpenCV_LIBS}
)

# Compile the supereight library
set(LIB_NAME "supereight2")
# Compiler options
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unknown-pragmas
    -Wno-unused-function
)
add_library(${LIB_NAME} STATIC
    "src/common/colour_utils.cpp"
    "src/common/system_utils.cpp"
    "src/common/image_utils.cpp"
    "src/common/perfstats.cpp"
    "src/common/str_utils.cpp"
    "src/common/yaml.cpp"
    "src/map/data.cpp"
    "src/map/io/mesh_io.cpp"
    "src/map/map.cpp"
    "src/map/octant.cpp"
    "src/map/preprocessor.cpp"
    "src/map/raycaster.cpp"
    "src/sensor/pinhole_camera.cpp"
    "src/sensor/sensor.cpp"
)
configure_file("include/se/supereight_config.hpp.in" "include/se/supereight_config.hpp" @ONLY)
# Set the C++ standard required by the library.
target_compile_features(${LIB_NAME} PUBLIC cxx_std_17)
target_include_directories(${LIB_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
    PUBLIC
      m
      Eigen3::Eigen
      Boost::boost
      ${TBB_IMPORTED_TARGETS}
      SRL::Projection
      ${GS_LIB_NAME}
)

# Find OpenMP, warn if disabled.
if(SE_OPENMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        target_link_libraries(${LIB_NAME} PUBLIC OpenMP::OpenMP_CXX)
        message(STATUS "Compiling with OpenMP support")
    else()
        message(WARNING "OpenMP not found. Performance may be terrible.")
    endif()
else()
    message(WARNING "Building without OpenMP. Performance may be terrible.")
endif()

if(("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") AND ("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 9.0))
    target_link_libraries(${LIB_NAME} PUBLIC stdc++fs)
elseif(("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") AND ("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 9.1))
    target_link_libraries(${LIB_NAME} PUBLIC stdc++fs)
endif()

# Add an alias so that the library can be used inside the build tree, e.g. when testing.
add_library(SRL::Supereight2 ALIAS ${LIB_NAME})

# This is required so that the exported target has the name Supereight2 and not supereight.
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME Supereight2)

# Compile the app
add_subdirectory(app)
