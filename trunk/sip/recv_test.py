#! /usr/bin/env python
from libserial import SerialPort
import errno
import sys

def main():
    serial_port = SerialPort( "/dev/ttyUSB11" )
    serial_port.Open( SerialPort.BAUD_115200,
                      SerialPort.CHAR_SIZE_DEFAULT,
                      SerialPort.PARITY_DEFAULT,
                      SerialPort.STOP_BITS_DEFAULT,
                      SerialPort.FLOW_CONTROL_HARD )
    try:
        while True:
            sys.stdout.write( serial_port.ReadByte() )
    except IOError, (errorNumber, errorMessage):
        if ( errno.EINTR == errorNumber ):
            print
            print "Ignoring EINTR."
            pass
        else:
            raise
    except KeyboardInterrupt:
        sys.exit(0)
	
if __name__ == "__main__":
	main()
