#! /usr/bin/env python
import binascii
import libserial
serial_port = libserial.SerialPort( libserial.std.string ( "/dev/ttyUSB0" ) )
serial_port.Open( libserial.SerialPort.BAUD_115200 )
while True:
    for i in range( 65, 91 ):
        serial_port.WriteByte( chr(i) )
    
