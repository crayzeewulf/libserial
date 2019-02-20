/******************************************************************************
 * @file UnitTests.h                                                          *
 * @copyright (C) 2004-2018 LibSerial Development Team. All rights reserved.  *
 * crayzeewulf@gmail.com                                                      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright          *
 *    notice, this list of conditions and the following disclaimer.           *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 *    notice, this list of conditions and the following disclaimer in         *
 *    the documentation and/or other materials provided with the              *
 *    distribution.                                                           *
 * 3. Neither the name PX4 nor the names of its contributors may be           *
 *    used to endorse or promote products derived from this software          *
 *    without specific prior written permission.                              *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS          *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE             *
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,        *
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,       *
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS      *
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED         *
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT                *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN          *
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE            *
 * POSSIBILITY OF SUCH DAMAGE.                                                *
 *****************************************************************************/

#pragma once

#include "libserial/SerialPort.h"
#include "libserial/SerialPortConstants.h"
#include "libserial/SerialStream.h"

#include <gtest/gtest.h>
#include <mutex>
#include <stdlib.h>
#include <sys/ioctl.h>

/**
 * @namespace Libserial
 */
namespace LibSerial
{
    /**
     * @var Default Serial Port 1.
     */
    constexpr const char* const SERIAL_PORT_1 = "/dev/ttyUSB0" ;

    /**
     * @var Default Serial Port 2.
     */
    constexpr const char* const SERIAL_PORT_2 = "/dev/ttyUSB1" ;

    /**
     * @var The number of iterations to perform on each unit test.
     */
    constexpr int TEST_ITERATIONS = 10 ;


    class UnitTests : public ::testing::Test
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit UnitTests() ;

        /**
         * @brief Default Destructor.
         */
        virtual ~UnitTests() ;

        /**
         * @brief Gets the time since epoch in milliseconds.
         * @return Returns the time since epoch in milliseconds
         */
        size_t getTimeInMilliSeconds() ;
        
        /**
         * @brief Gets the time since epoch in microseconds.
         * @return Returns the time since epoch in microseconds
         */
        size_t getTimeInMicroSeconds() ;

    protected:

        /**
         * @brief Tests for correct functionality for writing data from a serial stream object and reading that data from a serial port object..
         */
        void testSerialStreamToSerialPortReadWrite() ;

        /**
         * @var Failure rate of serial communications being tracked in the threaded tests.
         */
        size_t failureRate = 0 ;

        /**
         * @var Loop count variable.
         */
        size_t loopCount = 0 ;

        /**
         * @var Timeout to be used for test methods, (ms).
         */
        size_t timeOutMilliseconds = 250 ;

        /**
         * @var Time to allow the hardware read buffer to fill or empty (us).
         */
        unsigned int readBufferDelay = 20000 ;

        /**
         * @struct Standard baud rates.
         */
        std::vector<LibSerial::BaudRate> baudRates {} ;

        /**
         * @struct Standard character sizes.
         */
        std::vector<LibSerial::CharacterSize> characterSizes {} ;

        /**
         * @struct Standard flow control types.
         */
        std::vector<LibSerial::FlowControl> flowControlTypes {} ;

        /**
         * @struct Standard flow parity types.
         */
        std::vector<LibSerial::Parity> parityTypes {} ;

        /**
         * @struct Standard number of stop bits.
         */
        std::vector<LibSerial::StopBits> stopBits {} ;

        /**
         * @param Serial Stream instance 1 for unit testing applications.
         */
        LibSerial::SerialStream serialStream1 {} ;

        /**
         * @param Serial Stream instance 2 for unit testing applications.
         */
        LibSerial::SerialStream serialStream2 {} ;

        /**
         * @param Serial Port instance 1 for unit testing applications.
         */
        LibSerial::SerialPort serialPort1 {} ;

        /**
         * @param Serial Port instance 2 for unit testing applications.
         */
        LibSerial::SerialPort serialPort2 {} ;

        /**
         * @param String to store received data.
         */
        std::string readString1 {} ;

        /**
         * @param String to store received data.
         */
        std::string readString2 {} ;

        /**
         * @param String to store data to be written to the serial port.
         */
        std::string writeString1 {"Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)"} ;

        /**
         * @param String to store data to be written to the serial port.
         */
        std::string writeString2 {"The secret of the man who is universally interesting is that he is universally interested. - William Dean Howells"} ;

    private:

    } ;
}
