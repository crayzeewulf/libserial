#! /usr/bin/env python
import libserial
serial_port = libserial.SerialPort( libserial.std.string ( "/dev/ttyUSB1" ) )
serial_port.Open( libserial.SerialPort.BAUD_115200 )
while True:
    print serial_port.ReadByte(),
    
