cmake_minimum_required(VERSION 2.8.3)
set(name "MotionCapture")
project ( ${name} )

add_compile_options(-std=c++11)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# pointgrey camera sdk
if ( WIN32 )
    set(PTGREY_SDK_INCLUDE_DIR "C:/Program Files/Point Grey Research/Spinnaker/include")
    set(PTGREY_SDK_LIBRARY_DEBUG debug "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnakerd_v140.lib")
    set(PTGREY_SDK_LIBRARY_RELEASE  optimized "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")
else()
    set(PTGREY_SDK_INCLUDE_DIR "/usr/include/spinnaker")
    set(PTGREY_SDK_LIBRARY_DEBUG debug "/usr/lib/libSpinnaker.so")
    set(PTGREY_SDK_LIBRARY_RELEASE optimized "/usr/lib/libSpinnaker.so")
endif()
include_directories(${PTGREY_SDK_INCLUDE_DIR})



set(MY_HEADER_FILES
        include/single_calibrate.hpp
        )

set(MY_SOURCE_FILES
        src/Acquisition.cpp
        )



include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(${name}
        ${MY_SOURCE_FILES}
        ${MY_HEADER_FILES})

target_link_libraries(${name}
        ${OpenCV_LIBS}
        ${PTGREY_SDK_LIBRARY_DEBUG}
        ${PTGREY_SDK_LIBRARY_RELEASE}
        )