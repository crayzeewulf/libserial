/**
 *  @example serial_stream_read.cpp
 */

#include <libserial/SerialStream.h>

#include <cstdlib>
#include <iostream>
#include <unistd.h>

constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0" ;

/**
 * @brief This example demonstrates configuring a serial stream and 
 *        reading serial stream data.
 */
int main()
{
    using namespace LibSerial ;

    // Instantiate a SerialStream object.
    SerialStream serial_stream ;

    try
    {
        // Open the Serial Port at the desired hardware port.
        serial_stream.Open(SERIAL_PORT_1) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }

    // Set the baud rate of the serial port.
    serial_stream.SetBaudRate(BaudRate::BAUD_115200) ;

    // Set the number of data bits.
    serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

    // Turn off hardware flow control.
    serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

    // Disable parity.
    serial_stream.SetParity(Parity::PARITY_NONE) ;

    // Set the number of stop bits.
    serial_stream.SetStopBits(StopBits::STOP_BITS_1) ;

    // Wait for data to be available at the serial port.
    while(serial_stream.rdbuf()->in_avail() == 0) 
    {
        usleep(1000) ;
    }

    // Keep reading data from serial port and print it to the screen.
    while(serial_stream.IsDataAvailable()) 
    {
        // Variable to store data coming from the serial port.
        char data_byte ;

        // Read a single byte of data from the serial port.
        serial_stream.get(data_byte) ;

        // Show the user what is being read from the serial port.
        std::cout << data_byte ;

        // Wait a brief period for more data to arrive.
        usleep(1000) ;
    }

    // Successful program completion.
    std::cout << "Done." << std::endl ;
    return EXIT_SUCCESS ;
}
