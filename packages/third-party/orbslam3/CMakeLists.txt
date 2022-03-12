cmake_minimum_required(VERSION 2.8.3)
project(orbslam3_ros)

add_subdirectory(ORB_SLAM3)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  message_filters
)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

catkin_package(
    LIBRARIES ORB_SLAM3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ORB_SLAM3
  ORB_SLAM3/include
  ORB_SLAM3/include/CameraModels
  ORB_SLAM3/Examples_old/ROS/ORB_SLAM3
  ORB_SLAM3/Thirdparty/Sophus
  ${catkin_INCLUDE_DIRS}
)

add_executable(OrbSlam3Mono ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/src/ros_mono.cc)
target_link_libraries(OrbSlam3Mono ORB_SLAM3 ${catkin_LIBRARIES})

add_executable(OrbSlam3MonoIniterial ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/src/ros_mono_inertial.cc)
target_link_libraries(OrbSlam3MonoIniterial ORB_SLAM3 ${catkin_LIBRARIES})

add_executable(OrbSlam3Rgbd ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/src/ros_rgbd.cc)
target_link_libraries(OrbSlam3Rgbd ORB_SLAM3 ${catkin_LIBRARIES})

add_executable(OrbSlam3Stereo ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc)
target_link_libraries(OrbSlam3Stereo ORB_SLAM3 ${catkin_LIBRARIES})

add_executable(OrbSlam3StereoInertial ORB_SLAM3/Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc)
target_link_libraries(OrbSlam3StereoInertial ORB_SLAM3 ${catkin_LIBRARIES})


#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS
  ORB_SLAM3

  OrbSlam3Mono
  OrbSlam3MonoIniterial
  OrbSlam3Rgbd
  OrbSlam3Stereo
  OrbSlam3StereoInertial

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY ORB_SLAM3/include
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_burro_experimental.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
