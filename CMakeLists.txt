project(TransparentMirror)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(PCL 1.8 REQUIRED)
find_package(Qt4 COMPONENTS QTCORE)
# find_package(Qt5Core)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${QT_INCLUDES})
link_directories(${QT_LIBRARIES})
# add_definitions(${QT_USE_FILE})
# include_directories(${Qt5Core_INCLUDES})
# link_directories(${Qt5Core_LIBRARIES})
# add_definitions(${Qt5Core_DEFINITIONS})

set(SRC_LIST main.cpp cvs.cpp)

add_executable(transparent_mirror ${SRC_LIST})
target_link_libraries(transparent_mirror ${PCL_LIBRARIES})
target_link_libraries(transparent_mirror ${QT_QTCORE_LIBRARY})
# target_link_libraries(testPCL ${Qt5Core_LIBRARIES})

