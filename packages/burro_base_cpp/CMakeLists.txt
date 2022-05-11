cmake_minimum_required(VERSION 2.8.3)
project(burro_base_cpp)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    message_generation
    std_msgs
    serial
)

## Generate messages in the 'msg' folder
#add_message_files(
#   DIRECTORY msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

catkin_package(DEPENDS message_runtime)


###########
## Build ##
###########

add_subdirectory(libftdi)
add_subdirectory(roboteq)

include_directories(
  include
  libftdi/include
  roboteq/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(RoboteqDemo nodes/demo_node.cpp)
target_link_libraries(RoboteqDemo roboteq_serial ${catkin_LIBRARIES})

#############
## Install ##
#############

#catkin_install_python(PROGRAMS
#  <programs>
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  RoboteqDemo

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
