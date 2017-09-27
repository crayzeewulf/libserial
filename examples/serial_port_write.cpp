/******************************************************************************
 *   @file serial_port_write.cpp                                              *
 *   @copyright (C) 2016 LibSerial Development Team                           *
 *                                                                            *
 *   This program is free software; you can redistribute it and/or modify     *
 *   it under the terms of the GNU Lessser General Public License as          *
 *   published by the Free Software Foundation; either version 2 of the       *
 *   License, or (at your option) any later version.                          *
 *                                                                            *
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU Lesser General Public License for more details.                      *
 *                                                                            *
 *   You should have received a copy of the GNU Lesser General Public         *
 *   License along with this program; if not, write to the                    *
 *   Free Software Foundation, Inc.,                                          *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                *
 *****************************************************************************/

#include <SerialPort.h>

#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace LibSerial;

/**
 * @brief This example reads the contents of a file and writes the entire 
 *        file to the serial port one character at a time. To use this
 *        example, simply utilize TestFile.txt or another file of your
 *        choosing as a command line argument.
 */
int main(int argc, char** argv)
{   
    // Determine if an appropriate number of arguments has been provided.
    if (argc < 2)
    {
        // Error message to the user.
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;

        // Exit the program if no input file argument has been given.
        return 1;
    }

    // Open the input file for reading. 
    std::ifstream input_file(argv[1]);

    // Determine if the input file argument is valid to read data from.
    if (!input_file.good()) 
    {
        std::cerr << "Error: Could not open file "
                  << argv[1] << " for reading." << std::endl;
        return 1;
    }

    // Instantiate a SerialPort object.
    SerialPort serial_port;

    // Open the Serial Port at the desired hardware port.
    serial_port.Open("/dev/ttyUSB1");

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE);
    
    // Set the number of stop bits.
    serial_port.SetNumberOfStopBits(StopBits::STOP_BITS_1);

    // Read characters from the input file and write them to the serial port. 
    std::cout << "Writing input file contents to the serial port." << std::endl;
    
    // Create a variable to store data from the input file and write to the serial port.
    char data_byte;

    while (input_file) 
    {
        // Read data from the input file.
        input_file.read(&data_byte, 1);

        // Write the data to the serial port.
        serial_port.WriteByte(data_byte);

        // Print to the terminal what is being written to the serial port.
        std::cout << data_byte;
    }

    // Successful program completion.
    std::cout << std::endl << "Done." << std::endl;
    return EXIT_SUCCESS;
}
