/**
 *  @example serial_port_read.cpp
 */

#include <SerialPort.h>

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

using namespace LibSerial;

/**
 * @brief This example demonstrates configuring a serial port and 
 *        reading serial data.
 */
int main()
{
    // Instantiate a SerialPort object.
    SerialPort serial_port;

    // Open the Serial Port at the desired hardware port.
    serial_port.Open("/dev/ttyUSB0");

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE);
    
    // Set the number of stop bits.
    serial_port.SetStopBits(StopBits::STOP_BITS_1);
    
    // Wait for data to be available at the serial port.
    while(!serial_port.IsDataAvailable()) 
    {
        usleep(1000);
    }

    // Timeout value in milliseconds to wait for data being read.
    size_t ms_timeout = 250;

    // Char variable to store data coming from the serial port.
    char data_byte;

    // Read one byte from the serial port and print it to the terminal.
    try
    {
        // Read a single byte of data from the serial port.
        serial_port.ReadByte(data_byte, ms_timeout);

        // Show the user what is being read from the serial port.
        std::cout << data_byte << std::flush;
    }
    catch (ReadTimeout)
    {
        std::cerr << "\nThe ReadByte() call has timed out." << std::endl;
    }

    // Wait a brief period for more data to arrive.
    usleep(1000);

    // Size of the char array.
    const size_t array_size = 128;

    // Char array to store data coming from the serial port.
    char read_buffer[array_size];

    // Clear the array contents.
    std::memset(read_buffer, 0, array_size);

    try
    {
        // Read as many bytes as are available during the timeout period.
        serial_port.Read(read_buffer, 0, ms_timeout);
    }
    catch (ReadTimeout)
    {
        std::cerr << "\nThe Read() call has timed out waiting for more data." << std::endl;
    }

    std::cout << read_buffer << std::flush;

    // Successful program completion.
    std::cout << "Done." << std::endl;
    return EXIT_SUCCESS;
}