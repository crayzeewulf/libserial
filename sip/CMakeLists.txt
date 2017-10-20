ADD_LIBRARY(libserial SHARED
    SerialPort.sip
	SerialPortConstants.sip
	exception.sip
	libserial.sip
	stdexcept.sip
	string.sip
	vector.sip 
)

TARGET_LINK_LIBRARIES(libserial
	libserial_static
)

SET_TARGET_PROPERTIES(libserial
	PROPERTIES
	PREFIX ""
	SUFFIX .so
)

# TODO - Generate the SIP output with CMake:
#ADD_CUSTOM_COMMAND(OUTPUT
#  ${CMAKE_CURRENT_BINARY_DIR}/SerialPort.h
#  COMMAND ${SIP_EXECUTABLE} -c ${CMAKE_CURRENT_BINARY_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.sip
#  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.sip
#  COMMENT "Processing ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.sip"
#  VERBATIM
#  )
