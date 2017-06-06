cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(TransparentMirror)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set(freenect2_DIR "/usr/local/freenect2/")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/freenect2/")


INCLUDE_DIRECTORIES("/usr/local/vtk-7.1.1/inlcude/")
set(VTK_LIBRARIES "/usr/local/vtk-7.1.1/lib/")
set(VTK_DIR "/usr/local/vtk-7.1.1/lib/")

SET(VTK_DIR "/usr/local/vtk-7.1.1//lib/cmake/vtk-7.1/" CACHE PATH "VTK directory override" FORCE)

set(CMAKE_MODULE_PATH "/usr/local/pcl-1.8.0/" ${CMAKE_MODULE_PATH})
find_package(PCL 1.8 REQUIRED)
set(CMAKE_MODULE_PATH "/usr/local/vtk-7.1.1/")
find_package(VTK 7.1 REQUIRED NO_MODULE)
set(CMAKE_MODULE_PATH "/usr/lib/cmake/eigen3" ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
# find_package(Qt4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_MODULE_PATH "/usr/local/Qt5.7.1/" ${CMAKE_MODULE_PATH})
find_package(Qt5Core)
# find_package(Qt5Widgets REQUIRED)
# INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDES})
# LINK_LIBRARIES(${Qt5Widgets_LIBRARIES})

INCLUDE_DIRECTORIES("/usr/include/eigen3/")
INCLUDE_DIRECTORIES("/usr/local/freenect2/include/")
LINK_DIRECTORIES("/usr/local/freenect2/lib/")
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES("/usr/local/vtk-7.1.1/lib/")
LINK_DIRECTORIES("/usr/local/pcl-1.8.0/lib/")
LINK_DIRECTORIES("/usr/local/Qt5.7.1/5.7/gcc_64/lib/")

include(${VTK_USE_FILE})
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

set(PCL_INCLUDE_DIRS "/usr/local/pcl-1.8.0/include/pcl-1.8/")
set(PCL_LIBRARY_DIRS "/usr/local/pcl-1.8.0/lib/")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



include_directories("/usr/local/Qt5.7.1/5.7/gcc_64/include/")
# add_definitions(${QT_DEFINITIONS})
# include(${QT_USE_FILE})
# include_directories(${QT_INCLUDES})
# link_directories(${QT_LIBRARIES})
# add_definitions(${QT_USE_FILE})
# include_directories(${Qt5Core_INCLUDES})
# link_directories(${Qt5Core_LIBRARIES})
# add_definitions(${Qt5Core_DEFINITIONS})

# INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

# qt5_wrap_cpp(QT_MOC gui/main_window.h)
# qt5_wrap_cpp(QT_MOC gui/main_window.cpp)
qt5_wrap_cpp(QT_MOC_HEADERS gui/main_window.h)
# qt4_wrap_cpp(QT_MOC gui/main_window.cpp)
# add_library(QT_ALONE_LIB SHARED gui/main_window.h gui/main_window.cpp)
# target_link_libraries(QT_ALONE_LIB Qt5::Widgets)
# target_link_libraries(QT_ALONE_LIB ${QT_LIBRARIES})
# add_executable(qt_test main.cpp gui/main_window.h gui/main_window.cpp)
# target_link_libraries(qt_test Qt5::Widgets)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR})

# add_custom_target(header ${HEADER_LIST})

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
    gui/main_window.cpp
)

# qt4_add_resources(QT_MOC_RESOURCES gui/main_window.cpp main.cpp)

cuda_add_executable(transparent_mirror ${SRC_LIST} ${QT_MOC_HEADERS})
# cuda_add_executable(transparent_mirror ${SRC_LIST})
# add_executable(transparent_mirror ${SRC_LIST})
# target_link_libraries(transparent_mirror ${PCL_LIBRARIES})
target_link_libraries(transparent_mirror pcl_common)
target_link_libraries(transparent_mirror pcl_search)
target_link_libraries(transparent_mirror pcl_octree)
target_link_libraries(transparent_mirror pcl_features)
target_link_libraries(transparent_mirror pcl_filters)
target_link_libraries(transparent_mirror pcl_segmentation)
target_link_libraries(transparent_mirror pcl_visualization)
target_link_libraries(transparent_mirror boost_system)
# target_link_libraries(transparent_mirror ${BOOST_LIBRARIES})
# target_link_libraries(transparent_mirror ${QT_QTCORE_LIBRARY})
target_link_libraries(transparent_mirror freenect2)
target_link_libraries(transparent_mirror ${OpenCV_LIBS})
target_link_libraries(transparent_mirror ${VTK_LIBRARIES})
target_link_libraries(transparent_mirror Qt5Core)
target_link_libraries(transparent_mirror Qt5Widgets)
# target_link_libraries(transparent_mirror ${QT_LIBRARIES})
# qt5_use_modules(transparent_mirror Widgets)
# target_link_libraries(transparent_mirror QT_ALONE_LIB)
# target_link_libraries(transparent_mirror Qt5::Widgets)
# target_link_libraries(testPCL ${Qt5Core_LIBRARIES})

