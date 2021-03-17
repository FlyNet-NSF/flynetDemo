cmake_minimum_required(VERSION 2.8)
project(MUFTSG)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(FreeImage REQUIRED)

#if (NOT(DEFINED HAS_RRD_TOP_LEVEL))
#	set (RRD_TOP_LEVEL_DIR ${CMAKE_SOURCE_DIR}/..)
#	include (${RRD_TOP_LEVEL_DIR}/CMakeCommon.txt)
#endif ()

if (WIN32)
  set (CUDA_SAMPLES_DIR "C:/ProgramData/NVIDIA Corporation/CUDA Samples/v8.0" CACHE PATH "Path to CUDA Samples Directory")
  set (NVTX_DIR ${CMAKE_SOURCE_DIR}/nvtx CACHE PATH "Path to NVTX files")
  set (NVTX_INCLUDE_DIR ${NVTX_DIR})
  set (NVTX_LIB_DIR ${NVTX_DIR})
else()
  set (CUDA_SAMPLES_DIR "/usr/local/cuda/samples" CACHE PATH "Path to CUDA Samples Directory")
  set (NVTX_INCLUDE_DIR "/usr/local/cuda/include" CACHE PATH "Path to NVTX include file")
  set (NVTX_LIB_DIR "/usr/local/cuda/lib64" CACHE PATH "Path to NVTX lib file")
endif()
include_directories(${CUDA_SAMPLES_DIR}/common/inc)
include_directories(${NVTX_INCLUDE_DIR})
link_directories(${NVTX_LIB_DIR})


find_package(CUDA REQUIRED)
if(WIN32)
  list(APPEND CUDA_NVCC_FLAGS "-arch=sm_20;-O2;-DVERBOSE")
  set (CUDA_LIBS)
#  set (NVTX_LIB nvToolsExt64_1)
else()
  list(APPEND CUDA_NVCC_FLAGS "-O2;-DVERBOSE")
  SET (CMAKE_CXX_FLAGS "-std=c++11")
  SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
  set (CUDA_LIBS RRCuda nvToolsExt)
  set (NVTX_LIB nvToolsExt)
endif()

include_directories(/usr/local/cuda/samples/7_CUDALibraries/common/;/usr/local/cuda/samples/common/inc)
#include_directories($ENV{HOME}/NVIDIA_CUDA-8.0_Samples/common/inc)

LINK_DIRECTORIES(/usr/local/cuda/samples/common/lib/linux/armv7l/)

set (FLUXLIB_FILES
   filelisting_RP.cxx
   filelisting_RP.h
   FluxCudaKernels.h
   FluxCudaKernels.cu
   flux_util.cxx
   flux_util.h
   flux.cxx
   flux.h
   #CCLTrackerLantz.cxx
   #CCLTrackerLantz.h
   #npp/NPPImage.cxx
   #npp/Image.h
)

#cuda_add_library(muFlux STATIC ${FLUXLIB_FILES})
#cuda_add_library(muFlux_SHARED SHARED ${FLUXLIB_FILES})
#target_link_libraries(muFlux_SHARED ${OpenCV_LIBS} ${NVTX_LIB}
set (NPP_CUDA_10_1
${CUDA_nppial_LIBRARY}
${CUDA_nppicc_LIBRARY}
${CUDA_nppicom_LIBRARY}
${CUDA_nppidei_LIBRARY}
${CUDA_nppif_LIBRARY}
${CUDA_nppig_LIBRARY}
${CUDA_nppim_LIBRARY}
${CUDA_nppist_LIBRARY}
${CUDA_nppisu_LIBRARY}
${CUDA_nppitc_LIBRARY}
${CUDA_nppc_LIBRARY}
)

set (NPP_CUDA_9
${CUDA_nppial_LIBRARY}
${CUDA_nppicc_LIBRARY}
${CUDA_nppicom_LIBRARY}
${CUDA_nppidei_LIBRARY}
${CUDA_nppif_LIBRARY}
${CUDA_nppig_LIBRARY}
${CUDA_nppim_LIBRARY}
${CUDA_nppist_LIBRARY}
${CUDA_nppisu_LIBRARY}
${CUDA_nppitc_LIBRARY}
)

cuda_add_executable(Motion_Module motion_main.cpp ${FLUXLIB_FILES})
target_link_libraries(Motion_Module ${OpenCV_LIBS} ${NVTX_LIB} ${CUDA_LIBRARIES}
 ${NPP_CUDA_10_1}
)
# ${NPP_CUDA_9} ${CUDA_nppc_LIBRARY} ${CUDA 	_nppi_LIBRARY}
message("npp" ${CUDA_nppi_LIBRARY})
