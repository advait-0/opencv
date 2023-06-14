if(NOT HAVE_LIBCAMERA AND PKG_CONFIG_FOUND)
  ocv_check_modules(LIBCAMERA libcamera)
  message(STATUS "libcamera found!")
  message(STATUS "libcamera include directories: ${LIBCAMERA_INCLUDE_DIRS}")
  message(STATUS "libcamera libraries: ${LIBCAMERA_LIBRARIES}")

  if(LIBCAMERA_FOUND)
    set(HAVE_LIBCAMERA TRUE)
    set(GSTREAMER_VERSION ${LIBCAMERA_VERSION})  # informational
    set(LIBCAMERA_LIBRARIES ${LIBCAMERA_LIBRARIES})
    set(GSTREAMER_INCLUDE_DIRS ${LIBCAMERA_INCLUDE_DIRS})
  endif()
endif()

if(HAVE_LIBCAMERA)
  ocv_add_external_target(libcamera "${LIBCAMERA_INCLUDE_DIRS}" "${LIBCAMERA_LIBRARIES}" "HAVE_LIBCAMERA")
endif()
