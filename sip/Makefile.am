SIPFILES = \
SerialPort.sip \
exception.sip \
libserial.sip \
stdexcept.sip \
string.sip \
vector.sip 

EXTRA_DIST = \
configure.py.in \
recv_test.py \
send_test.py \
$(SIPFILES)

all: libserial.so

libserial.so: Makefile.sip @top_srcdir@/src/libserial.la
	make -f Makefile.sip

Makefile.sip: configure.py $(SIPFILES) 
	python ./configure.py

clean-local: Makefile.sip
	make -f Makefile.sip clean

CLEANFILES = \
siplibserialcmodule.cpp \
siplibserialSerialPort.cpp \
siplibserialSerialPortNotOpen.cpp \
siplibserialSerialPortAlreadyOpen.cpp \
siplibserialSerialPortOpenFailed.cpp \
siplibserialSerialPortReadTimeout.cpp \
siplibserialstd.cpp \
siplibserialstdexception.cpp \
siplibserialstdinvalid_argument.cpp \
siplibserialstdlogic_error.cpp \
siplibserialstdruntime_error.cpp \
siplibserialstdstring.cpp \
siplibserialstdvector.cpp \
libserial.sbf \
sipAPIlibserial.h \
Makefile.sip 