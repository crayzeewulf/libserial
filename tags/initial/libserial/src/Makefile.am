
INCLUDES = -I@STLDIR@/include/stlport \
           -D_STREAM_COMPAT -I../nienet/`../config.guess`/include/

CFLAGS = -g
CXXFLAGS = -g 
LDFLAGS = -L@STLDIR@/lib -lstlport_gcc

lib_LTLIBRARIES = libserial.la

include_HEADERS = SerialStreamBuf.h \
	SerialStream.h

libserial_la_SOURCES = \
	SerialStreamBuf.cc \
	SerialStreamBuf.h \
	SerialStream.C \
	SerialStream.h 

