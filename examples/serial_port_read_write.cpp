/**
 *  @example serial_port_read_write.cpp
 */

#include <libserial/SerialPort.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>

constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0" ;
constexpr const char* const SERIAL_PORT_2 = "/dev/ttyUSB1" ;

/**
 * @brief This example demonstrates multiple methods to read and write
 *        serial stream data.
 */
int main()
{
    using namespace LibSerial ;

    // Instantiate two SerialPort objects.
    SerialPort serial_port_1 ;
    SerialPort serial_port_2 ;

    try
    {
        // Open the Serial Ports at the desired hardware devices.
        serial_port_1.Open(SERIAL_PORT_1) ;
        serial_port_2.Open(SERIAL_PORT_2) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }

    // Set the baud rates.
    serial_port_1.SetBaudRate(BaudRate::BAUD_115200) ;
    serial_port_2.SetBaudRate(BaudRate::BAUD_115200) ;

    // Set the number of data bits.
    serial_port_1.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;
    serial_port_2.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

    // Set the hardware flow control.
    serial_port_1.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;
    serial_port_2.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

    // Set the parity.
    serial_port_1.SetParity(Parity::PARITY_NONE) ;
    serial_port_2.SetParity(Parity::PARITY_NONE) ;
    
    // Set the number of stop bits.
    serial_port_1.SetStopBits(StopBits::STOP_BITS_1) ;
    serial_port_2.SetStopBits(StopBits::STOP_BITS_1) ;

    // Setup variables to store outgoing and incoming data.
    char write_byte_1 = 'a' ;
    char write_byte_2 = 'b' ;

    char read_byte_1  = ' ' ;
    char read_byte_2  = ' ' ;

    // Variables to store outgoing and incoming data.
    std::string write_string_1 =
        "\"Do what you can, with what you have, where you are.\" - Theodore Roosevelt" ;
    
    std::string write_string_2 =
        "\"Simplicity is prerequisite for reliability.\" - Edsger W. Dijkstra" ;

    std::string read_string_1 ;
    std::string read_string_2 ;

    // Print to the terminal what will take place next.
    std::cout << "\nUsing WriteByte() and ReadByte() for one byte of data:"
              << std::endl ;

    // Write a single byte of data to the serial ports.
    serial_port_1.WriteByte(write_byte_1) ;
    serial_port_2.WriteByte(write_byte_2) ;

    // Wait until the data has actually been transmitted.
    serial_port_1.DrainWriteBuffer() ;
    serial_port_2.DrainWriteBuffer() ;

    // Specify a timeout value (in milliseconds).
    size_t timeout_milliseconds = 25 ;

    try
    {
        // Read a single byte of data from the serial ports.
        serial_port_1.ReadByte(read_byte_1, timeout_milliseconds) ;
        serial_port_2.ReadByte(read_byte_2, timeout_milliseconds) ;
    }
    catch (const ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_byte_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_byte_2  << std::endl
              << std::endl ;

    std::cout << "\tSerial Port 2 sent:\t"     << write_byte_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_byte_1  << std::endl
              << std::endl ;

    // Print to the terminal what will take place next.
    std::cout << "Using Write() and Read() for a specified number of "
              << "bytes of data:" << std::endl ;

    // Write a string to each serial port.
    serial_port_1.Write(write_string_1) ;
    serial_port_2.Write(write_string_2) ;

    // Wait until the data has actually been transmitted.
    serial_port_1.DrainWriteBuffer() ;
    serial_port_2.DrainWriteBuffer() ;

    try
    {
        // Read the appropriate number of bytes from each serial port.
        serial_port_1.Read(read_string_1, write_string_2.size(), timeout_milliseconds) ;
        serial_port_2.Read(read_string_2, write_string_1.size(), timeout_milliseconds) ;
    }
    catch (const ReadTimeout&)
    {
        std::cerr << "The Read() call has timed out." << std::endl ;
    }

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_string_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_string_2  << std::endl
              << std::endl ;

    std::cout << "\tSerial Port 2 sent:\t"     << write_string_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_string_1  << std::endl
              << std::endl ;
    
    // Variable to hold user input.
    std::string user_input ;
    user_input.clear() ;
    
    // Print to the terminal what will take place next.
    std::cout << "Using Write() and ReadLine() to write a string and "
              << "read a line of data:" << std::endl << std::endl ;

    // Prompt the user for input.
    std::cout << R"(Enter something you would like to send over )"
              << R"(serial, (enter "Q" or "q" to quit): )" << std::flush ;
    
    while(true)
    {
        // Get input from the user.
        std::getline(std::cin, user_input) ;

        if (user_input == "q" ||
            user_input == "Q" ||
            user_input == "")
        {
            break ;
        }

        // Write the user input to the serial port.
        serial_port_1.Write(user_input + "\n") ;

        // Read the data transmitted from the corresponding serial port.
        serial_port_2.ReadLine(read_string_2) ;

        // Print to the terminal what was sent and what was received.
        std::cout << "\tSerial Port 1 sent:\t"     << user_input   << std::endl
                  << "\tSerial Port 2 received:\t" << read_string_2 << std::endl ;
    }

    // Close the serial ports and end the program.
    serial_port_1.Close() ;
    serial_port_2.Close() ;

    // Successful program completion.
    std::cout << "The example program successfully completed!" << std::endl ;
    return EXIT_SUCCESS ;
}
