
noinst_PROGRAMS = readport writeport
# noinst_PROGRAMS = readport writeport xmodem_rx xmodem_tx process_rope_command test_echo

readport_SOURCES = read_port.cpp

writeport_SOURCES = write_port.cpp

# xmodem_rx_SOURCES = xmodem_rx.cpp xmodem.cpp xmodem.h crc16.cpp crc16.h byte_xfer.cpp byte_xfer.h

# xmodem_tx_SOURCES = xmodem_tx.cpp xmodem.cpp xmodem.h crc16.cpp crc16.h byte_xfer.cpp byte_xfer.h

# test_echo_SOURCES = test_echo.cpp

# process_rope_command_SOURCES = process_rope_command.cpp

INCLUDES = -I@top_srcdir@/src

readport_LDADD = ../src/libserial.la
writeport_LDADD = ../src/libserial.la
# xmodem_rx_LDADD = ../src/libserial.la
# xmodem_tx_LDADD = ../src/libserial.la
# test_echo_LDADD = ../src/libserial.la
# process_rope_command_LDADD = ../src/libserial.la

