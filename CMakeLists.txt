cmake_minimum_required(VERSION 2.8.3)
set(name "MotionCapture")
set(calib "StereoCalib")
set(retrive "RetriveImagePairs")
project ( ${name} )

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# pointgrey camera sdk
set(PTGREY_SDK_INCLUDE_DIR "D:/software/SpinnakerSDK/include")
set(PTGREY_SDK_LIBRARY_DEBUG debug "D:/software/SpinnakerSDK/lib64/vs2015/Spinnakerd_v140.lib")
set(PTGREY_SDK_LIBRARY_RELEASE  optimized "D:/software/SpinnakerSDK/lib64/vs2015/Spinnaker_v140.lib")
set(TCADS_LIBRARY "${PROJECT_SOURCE_DIR}/lib/TcAdsDll.lib")

include_directories(${PTGREY_SDK_INCLUDE_DIR})
message(STATUS "    OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


set(MY_HEADER_FILES
        include/Acquisition.hpp
        include/CircularDelay.hpp
        include/Tracker.hpp
        include/DataProcess.hpp
	include/TcAdsDef.h
	include/TcAdsAPI.h
        include/DigitalFilters.h
        )

set(MY_SOURCE_FILES
        src/main.cpp
        src/DataProcess.cpp
        src/Tracker.cpp
        )



include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

add_executable(${name}
        ${MY_SOURCE_FILES}
        ${MY_HEADER_FILES})

add_executable(${calib} 
        src/StereoCalib.cpp)

add_executable(${retrive}
        include/Acquisition.hpp
        src/RetriveImagePairs.cpp)

target_link_libraries(${name}
        ${OpenCV_LIBS}
        ${PTGREY_SDK_LIBRARY_DEBUG}
	${TCADS_LIBRARY}
        ${PTGREY_SDK_LIBRARY_RELEASE}
)
target_link_libraries(${calib}
        ${OpenCV_LIBS}
)

target_link_libraries(${retrive}
        ${OpenCV_LIBS}
        ${PTGREY_SDK_LIBRARY_DEBUG}
        ${PTGREY_SDK_LIBRARY_RELEASE}
)