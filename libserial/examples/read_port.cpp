#ifndef _std_iostream_INCLUDED_
#    include <iostream>
#    define _std_iostream_INCLUDED_
#endif

#ifndef _SerialStream_h_
#    include <SerialStream.h>
#endif

int
main( int    argc,
      char** argv  )
{
    //
    // Open the serial port.
    //
    using namespace LibSerial ;
    SerialStream serial_port ;
    serial_port.Open( "/dev/usb/tts/0" ) ;
    if ( ! serial_port.good() ) {
        std::cerr << "Error: Could not open serial port." << std::endl ;
        exit(1) ;
    }
    //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    if ( ! serial_port.good() ) {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        exit(1) ;
    }

    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() ) {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        exit(1) ;
    }

    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() ) {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        exit(1) ;
    }

    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() ) {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }

    //
    // Turn on hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_HARD ) ;
    if ( ! serial_port.good() ) {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Do not skip whitespace characters while reading from the
    // serial port.
    //
    // serial_port.unsetf( std::ios_base::skipws ) ;
    //
    // Keep reading data from serial port and print it to the screen.
    //
    char ch ;
    while( serial_port.get(ch) ) {
        std::cerr.put(ch) ;
    }
    return EXIT_SUCCESS ;
}
