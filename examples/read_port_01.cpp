#include "SerialPort.h"
#include <iostream>
#include <unistd.h>
#include <cstdlib>

using namespace LibSerial;

int main()
{
    // A simple example using the SerialPort class.
    SerialPort serial_port;

    serial_port.Open("/dev/ttyUSB0");

    std::string my_string;

    serial_port.ReadLine(my_string);

    std::cout << my_string;

    return EXIT_SUCCESS;
}