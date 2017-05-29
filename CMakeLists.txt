project(TransparentMirror)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(PCL 1.8 REQUIRED)
find_package(VTK REQUIRED)
find_package(Qt4 REQUIRED)
find_package(CUDA REQUIRED)
# find_package(Qt5Core)
# find_package(Qt5Widgets)

set(CUDA_SEPARABLE_COMPILATION ON)

###
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()

add_definitions("-Dint_p_NULL=((int*)0)")

# Pass options to NVCC. Notice the compatability.
# set(CUDA_NVCC_FLAGS -O3 --std=c++11 -gencode arch=compute_60,code=sm_60;-G;-g)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC;")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")
# set(CUDA_NVCC_FLAGS -O3 --std=c++11 -gencode arch=compute_60,code=sm_60;-G;-g)
# set(CUDA_NVCC_FLAGS "--std=c++11")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${QT_INCLUDES})
link_directories(${QT_LIBRARIES})
# add_definitions(${QT_USE_FILE})
# include_directories(${Qt5Core_INCLUDES})
# link_directories(${Qt5Core_LIBRARIES})
# add_definitions(${Qt5Core_DEFINITIONS})

INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

FILE(GLOB_RECURSE HeadFiles "*.h" "*.cu")
add_custom_target(headerfile ${HeadFiles})

file(GLOB CUDA_FILES disance_measurer.h disance_measurer.cu)

set(HEADER_LIST
    *.h)

add_custom_target(header ${HEADER_LIST})

set(SRC_LIST 
    main.cpp 
    cvs.cpp 
    edge_detector.cpp 
    cvs_estimation.cpp 
    cvs_corresp_group.cpp
    ccn.cpp
    board_detector.cpp
    ccn_estimation.cpp
    ccn_corresp_group.cpp
    icf.cpp
    mesh.cpp
    distance_measurer.cu
    distance_measurer.h
    gui/main_window.cpp)

CUDA_COMPILE(cuda_o ${CUDA_FILES})
# CUDA_ADD_LIBRARY(cuda_o ${CUDA_FILES})
# add_library(${cuda_o})
# target_link_libraries(${cuda_o} pcl_gpu_containers)
# add_executable(transparent_mirror ${SRC_LIST} ${cuda_o})
cuda_add_executable(transparent_mirror ${SRC_LIST} ${cuda_o})
# cuda_add_executable(transparent_mirror ${SRC_LIST} ${CUDA_FILES})
target_link_libraries(transparent_mirror ${PCL_LIBRARIES})
target_link_libraries(transparent_mirror ${QT_QTCORE_LIBRARY})
# target_link_libraries(transparent_mirror cuda_o)
target_link_libraries(transparent_mirror ${CUDA_CUDA_LIBRARIES})
# target_link_libraries(testPCL ${Qt5Core_LIBRARIES})

