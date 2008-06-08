#! /usr/bin/env python
import libserial
import errno
import sys

def main():
    serial_port = libserial.SerialPort( libserial.std.string ( "/dev/ttyUSB1" ) )
    serial_port.Open( libserial.SerialPort.BAUD_115200,
                      libserial.SerialPort.CHAR_SIZE_DEFAULT,
                      libserial.SerialPort.PARITY_DEFAULT,
                      libserial.SerialPort.STOP_BITS_DEFAULT,
                      libserial.SerialPort.FLOW_CONTROL_HARD )
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
