ADD_LIBRARY(libserialmod SHARED
    SerialPort.sip
	SerialPortConstants.sip
	exception.sip
	libserial.sip
	stdexcept.sip
	string.sip
	vector.sip
)

SET_TARGET_PROPERTIES(libserialmod PROPERTIES PREFIX "")

TARGET_LINK_LIBRARIES(libserialmod
	libserial_static
)

INSTALL(TARGETS libserialmod
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# TODO - Generate the SIP output with CMake:
#ADD_CUSTOM_COMMAND(OUTPUT
#  ${CMAKE_CURRENT_BINARY_DIR}/SerialPort.h
#  COMMAND ${SIP_EXECUTABLE} -c ${CMAKE_CURRENT_BINARY_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.sip
#  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.sip
#  COMMENT "Processing ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.sip"
#  VERBATIM
#  )
