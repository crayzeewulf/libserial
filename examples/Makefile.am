AM_CXXFLAGS = -Weffc++

AM_CPPFLAGS = -I@top_srcdir@/src

noinst_PROGRAMS = read_port write_port read_port_01

read_port_SOURCES    = read_port.cpp
read_port_01_SOURCES = read_port_01.cpp
write_port_SOURCES   = write_port.cpp

read_port_LDADD    = ../src/libserial.la -lpthread
read_port_01_LDADD = ../src/libserial.la -lpthread
write_port_LDADD   = ../src/libserial.la -lpthread


# noinst_PROGRAMS = xmodem_rx xmodem_tx process_rope_command test_echo
# xmodem_rx_SOURCES = xmodem_rx.cpp xmodem.cpp xmodem.h crc16.cpp crc16.h byte_xfer.cpp byte_xfer.h
# xmodem_tx_SOURCES = xmodem_tx.cpp xmodem.cpp xmodem.h crc16.cpp crc16.h byte_xfer.cpp byte_xfer.h
# test_echo_SOURCES = test_echo.cpp
# process_rope_command_SOURCES = process_rope_command.cpp

# xmodem_rx_LDADD = ../src/libserial.la
# xmodem_tx_LDADD = ../src/libserial.la
# test_echo_LDADD = ../src/libserial.la
# process_rope_command_LDADD = ../src/libserial.la
