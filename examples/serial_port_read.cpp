#include <SerialPort.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace LibSerial;

int main()
{
    // Instantiate a SerialPort object and open the serial port.
    SerialPort serial_port;

    serial_port.Open("/dev/ttyUSB0");

    // Check that the serial stream has opened correctly.
    if (!serial_port.IsOpen()) 
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                  << "Error: Could not open serial port." 
                  << std::endl;
        exit(1);
    }

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate( BaudRate::BAUD_115200 );

    // Set the number of data bits.
    serial_port.SetCharacterSize( CharacterSize::CHAR_SIZE_8 );

    // Turn off hardware flow control.
    serial_port.SetFlowControl( FlowControl::FLOW_CONTROL_NONE );

    // Disable parity.
    serial_port.SetParity( Parity::PARITY_NONE );
    
    // Set the number of stop bits.
    serial_port.SetNumberOfStopBits( StopBits::STOP_BITS_1 );
    
    // Wait for some data to be available at the serial port.
    while(!serial_port.IsDataAvailable()) 
    {
        usleep(100);
    }

    // Keep reading data from serial port and print it to the screen.
    while(serial_port.IsDataAvailable()) 
    {
        unsigned char nextByte;
        unsigned int msTimeout = 50;

        serial_port.ReadByte(nextByte, msTimeout);
        std::cerr << std::hex << static_cast<int>(nextByte) << " ";
        usleep(100);
    }

    std::cerr << std::endl;
    return EXIT_SUCCESS;
}