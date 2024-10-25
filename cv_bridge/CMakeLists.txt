cmake_minimum_required(VERSION 3.0.2)
project(cv_bridge)

find_package(catkin REQUIRED COMPONENTS rosconsole sensor_msgs)


if(NOT ANDROID)
  find_package(PythonLibs 2)
  set(PYTHON_NUMPY_INCLUDE_DIR ~/.local/lib/python3.6/site-packages/numpy/core/include)
  set(PYTHON_INCLUDE_PATH /usr/include/python3.6)
  set(PYTHON_LIBRARIES /usr/lib/python3.6)

  if(PYTHONLIBS_VERSION_STRING VERSION_LESS "3.8")
    # Debian Buster
    find_package(Boost REQUIRED python3)
  else()
    # Ubuntu Focal
    find_package(Boost REQUIRED python3)
  endif()
else()
find_package(Boost REQUIRED)
endif()

# Print some message showing some of them
message("##############################################")
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBRARIES}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message("##############################################")
 
# Print some message showing some of them
message("##############################################")
message(STATUS "PYTHON library status:")
message(STATUS "    PYTHON_VERSION_MAJOR: ${PYTHON_VERSION_MAJOR}")
message(STATUS "    PYTHON_VERSION_MINOR: ${PYTHON_VERSION_MINOR}")
message(STATUS "    include path: ${PYTHON_NUMPY_INCLUDE_DIR}")

set(_opencv_version 4)
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rosconsole sensor_msgs
  DEPENDS OpenCV
  CFG_EXTRAS cv_bridge-extras.cmake
)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

if(NOT ANDROID)
add_subdirectory(python)
endif()
add_subdirectory(src)
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
