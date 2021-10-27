cmake_minimum_required(VERSION 2.8.3)
project(uwb-imu-positioning)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(
    OpenCV
    NO_DEFAULT_PATH
    PATHS /usr/local/share/OpenCV /opt/ros/kinetic/share/OpenCV-3.2.0
)

set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common_msgs REQUIRED
  visualization_msgs
  nav_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
)

# pay attention that Pangolin should be modified and compiled with boost and c++99.

find_library(Reflexxes_LIBRARIES flexxes ${PROJECT_SOURCE_DIR}/exLib/Reflexxes)
set(Relfexxes_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/exLib/Reflexxes/Reflexxes)

find_package(Boost REQUIRED COMPONENTS system thread iostreams chrono)

find_package(Eigen3)
find_package(Ceres REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS common_msgs roscpp rospy std_msgs tf
  DEPENDS
)

###########
## Build ##
###########



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/exLib
  ${Eigen_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


# with C++99, please use ORB_SLAM2_boost
set(Library_Set
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CERES_LIBRARIES}
)



## Declare a C++ library
file(GLOB_RECURSE HEADERS */*.hpp */*.h)

## Declare a C++ executable
add_executable(uwb-imu-positioning
    ${HEADERS} # for qtcreator...

    src/localization_node.cc
    src/pose_local_parameterization.cc
    src/uwb_init_loc.cc
    src/uwb.cc

)

## Specify libraries to link a library or executable target against

target_link_libraries(uwb-imu-positioning
    ${Library_Set}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(uwb-imu-positioning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
