cmake_minimum_required(VERSION 3.5)
project(ssm_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# Set c11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  geometry_msgs
  std_msgs
)

pkg_search_module(GLIB REQUIRED glib-2.0)
# Note: If using catkin, Python 2 is found since it points
# to the Python libs installed with the ROS distro
if (NOT CATKIN_DEVEL_PREFIX)
	find_package(PythonInterp 3 REQUIRED)
else()
	find_package(roscpp REQUIRED)
	message(STATUS "${roscpp_version}")
	if(${roscpp_VERSION}  VERSION_LESS "1.15.0")
		find_package(PythonInterp REQUIRED)
	else()
		find_package(PythonInterp 3 REQUIRED)
	endif()
endif()

pkg_check_modules(OGRE OGRE)

if("${GAZEBO_VERSION}" VERSION_LESS "8.0")
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
else()
  include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()


# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()


set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
  # ${OpenCV_INCLUDE_DIRS}
  # ${OpticalFlow_INCLUDE_DIRS}
  # ${TinyXML_INCLUDE_DIRS}
  )

include_directories(${catkin_INCLUDE_DIRS})

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  # ${OpenCV_LIBRARIES}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_LIBRARY_DIRS}
  )


set(ssm_msgs
  msgs/VectorVisual.proto
  msgs/SphericalVector.proto
  msgs/MultiplevectorVisual.proto
  msgs/dynamic_state.proto
)

set(msgs msgs/double.proto)


PROTOBUF_GENERATE_CPP(SSM_PROTO_SRCS SSM_PROTO_HDRS ${ssm_msgs})
PROTOBUF_GENERATE_CPP(SRC_PROTO_SRCS SRC_PROTO_HDRS ${msgs})

add_library(ssm_msgs SHARED ${SSM_PROTO_SRCS})
link_libraries(ssm_msgs)

add_library(msgs SHARED ${SRC_PROTO_SRCS})
link_libraries(msgs)

add_library(ForceVisual SHARED src/force_visual.cc)
list(APPEND plugins ForceVisual)

add_library(MultiplevectorVisual SHARED src/multivector_visual.cc)
list(APPEND plugins MultiplevectorVisual)

add_library(SimpleThruster SHARED src/simple_thruster.cc)
list(APPEND plugins SimpleThruster)

add_library(ThrustVectoring SHARED src/thrust_vectoring.cc)
list(APPEND plugins ThrustVectoring)

add_library(PosePublisher SHARED src/pose_publisher.cc)
list(APPEND plugins PosePublisher)

add_library(Aerodynamic SHARED src/aerodynamic.cc)
list(APPEND plugins Aerodynamic)

add_library(DetachableJoint SHARED src/detachable_joint.cc)
list(APPEND plugins DetachableJoint)

add_library(InitialPosition SHARED src/initial_position.cc)
list(APPEND plugins InitialPosition)

foreach(plugin ${plugins})
  add_dependencies(${plugin} ssm_msgs)
  add_dependencies(${plugin} msgs)
  target_link_libraries(${plugin} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  
endforeach()

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
  DEPENDS 
    roscpp 
    gazebo_ros 
)