cmake_minimum_required(VERSION 2.8.3)
project(mbot_visual_servoing_imagebased)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Set optimization
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find BOOST
find_package(Boost REQUIRED system thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS})
if(NOT Boost_FOUND)
    message(ERROR " Boost not found!")
endif(NOT Boost_FOUND)

# Find OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(ERROR " OpenCV not found!")
endif(NOT OpenCV_FOUND)

#Find Realsense2
find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

find_package(catkin REQUIRED
        rosconsole
        roscpp
        geometry_msgs
        sensor_msgs
        mbot_perception_msgs
        visp_ros
        tf
        darknet_ros_py
        message_filters
        cv_bridge
        std_msgs
        message_generation
        actionlib
        actionlib_msgs
        )

add_action_files(DIRECTORY ros/action FILES GraspObject.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS
        rosconsole
        roscpp
        geometry_msgs
        sensor_msgs
        mbot_perception_msgs
        actionlib
        actionlib_msgs
        )

include_directories(
        ros/include
        ${catkin_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )


### EXECUTABLES
add_executable(action_client_vs ros/src/action_client_vs.cpp)
target_link_libraries(action_client_vs ${catkin_LIBRARIES} ${visp_ros_LIBRARIES} ${realsense2_LIBRARY} ${OpenCV_LIBRARIES})
add_dependencies(action_client_vs ${mbot_visual_servoing_imagebased_EXPORTED_TARGETS})

add_executable(action_server_vs ros/src/action_server_vs.cpp)
target_link_libraries(action_server_vs ${catkin_LIBRARIES} ${visp_ros_LIBRARIES} ${realsense2_LIBRARY} ${OpenCV_LIBRARIES})
add_dependencies(action_server_vs ${mbot_visual_servoing_imagebased_EXPORTED_TARGETS})

