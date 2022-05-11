
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(roboteq src/roboteq.cpp)
add_library(roboteq_serial src/roboteq_serial.cpp)

target_link_libraries(roboteq_serial roboteq)