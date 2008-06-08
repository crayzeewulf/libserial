#! /usr/bin/env python
import sys
import libserial

def main():
    serial_port = libserial.SerialPort( "/dev/ttyUSB0" )
    serial_port.Open( libserial.SerialPort.BAUD_115200,
                      libserial.SerialPort.CHAR_SIZE_DEFAULT,
                      libserial.SerialPort.PARITY_DEFAULT,
                      libserial.SerialPort.STOP_BITS_DEFAULT,
                      libserial.SerialPort.FLOW_CONTROL_HARD )
    try:
        while True:
            for i in range( 65, 91 ):
                serial_port.WriteByte( chr(i) )
                sys.stdout.write( chr(i) )
    except KeyboardInterrupt:
        sys.exit(0)    
        
if __name__ == "__main__":
    main()
