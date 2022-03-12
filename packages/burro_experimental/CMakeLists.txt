cmake_minimum_required(VERSION 2.8.3)
project(burro_experimental)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-Werror=return-type")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  burro_cpp
)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

find_package(GTSAM REQUIRED)
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  message(STATUS "Will link with GTSAM Debug binaries")
  set(GTSAM_LIBRARIES "${GTSAM_LIBRARIES_DEBUG}")
endif()

list(APPEND GTSAM_LIBRARIES tbb tbbmalloc)


catkin_package(
    LIBRARIES burro_experimental
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

add_library(ml src/ml/kdtree.cpp)

add_executable(DbscanExample nodes/dbscan_example.cpp)
target_link_libraries(DbscanExample ml util)


#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS
  ml

  DbscanExample

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/burro_exp/
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
