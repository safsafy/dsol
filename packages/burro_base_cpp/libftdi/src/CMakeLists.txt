# Includes
include_directories(BEFORE ${CMAKE_BINARY_DIR}/libftdi/include
                     ${CMAKE_SOURCE_DIR}/libftdi/include
                     )

# Version information
set(SNAPSHOT_VERSION "burro-1.5")

configure_file(ftdi_version_i.h.in "${CMAKE_BINARY_DIR}/libftdi/include/ftdi_version_i.h" @ONLY)

# Targets
set(c_sources     ${CMAKE_CURRENT_SOURCE_DIR}/ftdi.c CACHE INTERNAL "List of c sources" )
set(c_headers     ${CMAKE_SOURCE_DIR}/libftdi/include/ftdi.h CACHE INTERNAL "List of c headers" )

add_library(ftdi1 SHARED ${c_sources})

math(EXPR VERSION_FIXUP "${MAJOR_VERSION} + 1")    # Compatibility with previous releases
set_target_properties(ftdi1 PROPERTIES VERSION ${VERSION_FIXUP}.${MINOR_VERSION}.0 SOVERSION 2)
# Prevent clobbering each other during the build
set_target_properties ( ftdi1 PROPERTIES CLEAN_DIRECT_OUTPUT 1 )


# Dependencies
target_link_libraries(ftdi1 ${LIBUSB_LIBRARIES})


install ( TARGETS ftdi1
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib${LIB_SUFFIX}
          ARCHIVE DESTINATION lib${LIB_SUFFIX}
        )

if ( STATICLIBS )
  add_library(ftdi1-static STATIC ${c_sources})
  target_link_libraries(ftdi1-static ${LIBUSB_LIBRARIES})
  set_target_properties(ftdi1-static PROPERTIES OUTPUT_NAME "ftdi1")
  set_target_properties(ftdi1-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
  install ( TARGETS ftdi1-static
            ARCHIVE DESTINATION lib${LIB_SUFFIX}
            COMPONENT staticlibs
          )
endif ()

install ( FILES ${c_headers}
          DESTINATION include/${PROJECT_NAME}
          COMPONENT headers
        )
