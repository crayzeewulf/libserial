#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace LibSerial;

int main()
{
    // Instantiate a SerialStream object then open the serial port.
    SerialStream serial_stream;

    serial_stream.Open("/dev/ttyUSB1");
    
    // Check that the serial stream has opened correctly.
    if (!serial_stream.good()) 
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port." 
                  << std::endl;
        exit(1);
    }

    // Set the baud rate of the serial port.
    serial_stream.SetBaudRate( BaudRate::BAUD_115200 );

    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not set the baud rate." << std::endl;
        exit(1) ;
    }

    // Set the number of data bits.
    serial_stream.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );

    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not set the character size." << std::endl;
        exit(1) ;
    }

    // Turn off hardware flow control.
    serial_stream.SetFlowControl( FlowControl::FLOW_CONTROL_NONE );

    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not use hardware flow control."
                  << std::endl;
        exit(1);
    }

    // Disable parity.
    serial_stream.SetParity( Parity::PARITY_NONE );
    
    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not disable the parity." << std::endl;
        exit(1);
    }

    // Set the number of stop bits.
    serial_stream.SetNumberOfStopBits( StopBits::STOP_BITS_1 );
    
    if (!serial_stream.good()) 
    {
        std::cerr << "Error: Could not set the number of stop bits."
                  << std::endl;
        exit(1);
    }

    // Do not skip whitespace characters while reading from the serial port.
    // serialStream.unsetf(std::ios_base::skipws);

    // Wait for some data to be available at the serial port.
    while(serial_stream.rdbuf()->in_avail() == 0) 
    {
        usleep(100);
    }

    // Keep reading data from serial port and print it to the screen.
    while(serial_stream.rdbuf()->in_avail() > 0) 
    {
        char nextByte;
        serial_stream.get(nextByte);
        std::cerr << std::hex << static_cast<int>(nextByte) << " ";
        usleep(100);
    }

    std::cerr << std::endl;
    return EXIT_SUCCESS;
}
