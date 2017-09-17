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
