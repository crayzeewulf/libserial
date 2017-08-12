ADD_LIBRARY(libSerial STATIC
    SerialPort.cpp
    SerialStream.cc
    SerialStreamBuf.cc
)

SET_TARGET_PROPERTIES(libSerial
	PROPERTIES
	PREFIX ""
	SUFFIX .la
)
