#! /usr/bin/env python
from libserial import SerialPort
from time      import sleep
import optparse
import logging

def create_options_parser():
    """Create a parser to extract value from command line options."""
    parser = optparse.OptionParser( usage = "%prog [opts]" )
    parser.add_option( "-d", "--device",
                       action  = "store",
                       type    = "string",
                       help    = "Serial port device",
                       default = "/dev/ttyS0" )
    return parser


def main():
    #
    # Parse the command line options specified by the user.
    #
    parser = create_options_parser()
    (options, arguments) = parser.parse_args()
    #
    logging.info( "Opening serial port " + options.device )
    serial_port = SerialPort( options.device )
    serial_port.Open( SerialPort.BAUD_DEFAULT, 
                      SerialPort.CHAR_SIZE_DEFAULT, 
                      SerialPort.PARITY_DEFAULT,
                      SerialPort.STOP_BITS_DEFAULT,
                      SerialPort.FLOW_CONTROL_NONE )
    while True:
        #
        serial_port.SetRts( True )
        serial_port.SetDtr( False )
        #
        assert ( True  == serial_port.GetRts() )
        assert ( False == serial_port.GetDtr() )
        #
        sleep(1) 
        #
        serial_port.SetRts( False )
        serial_port.SetDtr( True )
        #
        assert ( False == serial_port.GetRts() )
        assert ( True  == serial_port.GetDtr() )
        #
        sleep(1) 
###############################################################################
# The script starts here.
###############################################################################

if __name__ == "__main__":
    try:
        logging.basicConfig( level  = logging.DEBUG,
                             format = '%(asctime)s %(levelname)s %(message)s' )
        main()
    except SystemExit:
        raise
    except:
        print \
"""An internal error occured.  Please report all the output from the program,
including the following traceback.
"""
        raise
