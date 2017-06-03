project(TransparentMirror)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set(freenect2_DIR "/usr/local/freenect2/")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/freenect2/")

find_package(PCL 1.8 REQUIRED)
# find_package(VTK REQUIRED)
find_package(Qt4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)
# include(FindCUDA)
# find_package(Qt5Core)
# find_package(Qt5Widgets)

INCLUDE_DIRECTORIES("/usr/local/freenect2/include/")
LINK_DIRECTORIES("/usr/local/freenect2/lib/")
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Fix the bug in VTK which could cause compilation error.
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

# Pass options to NVCC. Notice the compatability.
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -O3 -std=c++11 -gencode arch=compute_60,code=sm_60;-G;-g)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -O3 -gencode arch=compute_61,code=sm_61)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-Xcompiler;-fPIC;")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${QT_INCLUDES})
link_directories(${QT_LIBRARIES})
# add_definitions(${QT_USE_FILE})
# include_directories(${Qt5Core_INCLUDES})
# link_directories(${Qt5Core_LIBRARIES})
# add_definitions(${Qt5Core_DEFINITIONS})

# INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

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
    kinect2_grabber.cpp
    gui/main_window.cpp)

cuda_add_executable(transparent_mirror ${SRC_LIST})
target_link_libraries(transparent_mirror ${PCL_LIBRARIES})
target_link_libraries(transparent_mirror ${QT_QTCORE_LIBRARY})
target_link_libraries(transparent_mirror freenect2)
target_link_libraries(transparent_mirror ${OpenCV_LIBS})
# target_link_libraries(testPCL ${Qt5Core_LIBRARIES})

