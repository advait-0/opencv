cmake_minimum_required(VERSION 3.5)

get_filename_component(OpenCV_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
include("${OpenCV_SOURCE_DIR}/cmake/OpenCVPluginStandalone.cmake")

# scan dependencies
set(WITH_LIBCAMERA ON)
include("${OpenCV_SOURCE_DIR}/modules/videoio/cmake/init.cmake")

set(OPENCV_PLUGIN_DEPS core imgproc imgcodecs)
ocv_create_plugin(videoio "opencv_videoio_libcamera" "ocv.3rdparty.libcamera" "libcamera" "src/cap_libcamera.cpp")

message(STATUS "Using libcamera: ${LIBCAMERA_VERSION}")
