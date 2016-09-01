cmake_minimum_required(VERSION 2.8)
project(rt_estimation)


set(SOURCES_FILES_CPP
    ./src/cameras/Cam.cpp
    ./src/cameras/OmniCam.cpp
    ./src/matching/autoMatching.cpp
    ./src/matching/manualMatching.cpp
    ./src/rtEstimation/rtEstimation.cpp
    ./src/utils/experimentManager.cpp
)

ADD_DEFINITIONS (
    -std=c++11
)

#Look for OpenCV Lib
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIR}/include)
include_directories(${OpenCV_DIR}/include/opencv2)
link_directories(${OpenCV_DIR}/lib ${OpenCV_DIR}/3rdparty/lib)
FIND_PACKAGE(OpenMVG REQUIRED)
INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})

#Use OpenCV
set(EXTRA_LIBS
    ${OpenCV_LIBS}
    ${OPENMVG_LIBRARIES}
    vlsift
)

#Build
add_executable(rt_estimation ./src/main.cpp ${SOURCES_FILES_CPP})
target_link_libraries(rt_estimation ${EXTRA_LIBS})

add_executable(point_marker ./src/pointMarker.cpp ${SOURCES_FILES_CPP})
target_link_libraries(point_marker ${EXTRA_LIBS})
