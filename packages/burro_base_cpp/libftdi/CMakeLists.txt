set(MAJOR_VERSION 1)
set(MINOR_VERSION 5)
set(VERSION_STRING ${MAJOR_VERSION}.${MINOR_VERSION})
set(VERSION ${VERSION_STRING})

find_package ( USB1 REQUIRED )
find_package( Boost REQUIRED )

include_directories ( ${LIBUSB_INCLUDE_DIR} )
include_directories ( include )

add_subdirectory(src)
add_subdirectory(ftdipp)

add_executable(FindFtdiDevices tests/find_ftdi_devices.cpp)
target_link_libraries(FindFtdiDevices ftdipp1)