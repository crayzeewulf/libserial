#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

int
main( int    argc,
      char** argv  )
{
    (void) argc; // Quiet compiler warning.
    (void) argv; // Quiet compiler warning.

    //
    // Open the serial port.
    //
    using namespace LibSerial ;
    SerialStream serial_stream ;
    serial_stream.Open( "/dev/ttyUSB0" ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port." 
                  << std::endl ;
        exit(1) ;
    }
    //
    // Set the baud rate of the serial port.
    //
    serial_stream.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_stream.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl ;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_stream.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl ;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_stream.SetNumOfStopBits( 1 ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Turn off hardware flow control.
    //
    serial_stream.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_stream.good() ) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl ;
        exit(1) ;
    }
    //
    // Do not skip whitespace characters while reading from the
    // serial port.
    //
    // serial_stream.unsetf( std::ios_base::skipws ) ;
    //
    // Wait for some data to be available at the serial port.
    //
    while( serial_stream.rdbuf()->in_avail() == 0 ) 
    {
        usleep(100) ;
    }
    //
    // Keep reading data from serial port and print it to the screen.
    //
    while( serial_stream.rdbuf()->in_avail() > 0  ) 
    {
        char next_byte;
        serial_stream.get(next_byte);
        std::cerr << std::hex << static_cast<int>(next_byte) << " ";
        usleep(100) ;
    } 
    std::cerr << std::endl ;
    return EXIT_SUCCESS ;
}
