#include "SerialPort.h"
#include <iostream>
#include <unistd.h>
#include <cstdlib>

int main()
{
    // A simple example using the SerialPort class.
    SerialPort serial_port("/dev/ttyUSB0");

    serial_port.Open();

    std::cout << serial_port.ReadLine();

    return EXIT_SUCCESS;
}