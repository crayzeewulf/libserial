/******************************************************************************
 * @file UnitTests.cpp                                                        *
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

#include "UnitTests.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace LibSerial;

UnitTests::UnitTests()
{
    // Empty
}


UnitTests::~UnitTests()
{
    // Empty
}

size_t
UnitTests::getTimeInMilliSeconds()
{
    std::chrono::high_resolution_clock::duration timeNow = 
        std::chrono::high_resolution_clock::now().time_since_epoch() ;

    return static_cast<size_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(timeNow).count()
    ) ;
}

size_t
UnitTests::getTimeInMicroSeconds()
{
    std::chrono::high_resolution_clock::duration timeNow = 
        std::chrono::high_resolution_clock::now().time_since_epoch() ;

    return static_cast<size_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(timeNow).count()
    ) ;
}

void
UnitTests::testSerialStreamToSerialPortReadWrite()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    ASSERT_TRUE(serialPort1.IsOpen()) ;

    serialStream1.Open(SERIAL_PORT_2) ;
    ASSERT_TRUE(serialStream1.IsOpen()) ;

    const auto baud_rate = BaudRate::BAUD_115200 ;
    serialPort1.SetBaudRate(baud_rate) ;
    serialStream1.SetBaudRate(baud_rate) ;

    const auto baudRate1 = serialPort1.GetBaudRate() ;
    const auto baudRate2 = serialStream1.GetBaudRate() ;

    ASSERT_EQ(baudRate1, baud_rate) ;
    ASSERT_EQ(baudRate2, baud_rate) ;

    serialStream1 << writeString1 << std::endl;
    serialPort1.ReadLine(readString1, '\n', timeOutMilliseconds) ;

    ASSERT_EQ(readString1, writeString1 + '\n') ;
    ASSERT_EQ(readString1.size(), writeString1.size() + 1) ;

    serialPort1.Write(writeString2 + '\n') ;
    serialPort1.DrainWriteBuffer() ;
    getline(serialStream1, readString2) ;

    ASSERT_EQ(readString2, writeString2) ;

    serialPort1.Close() ;
    serialStream1.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialStream1.IsOpen()) ;
}

TEST_F(UnitTests, testSerialStreamToSerialPortReadWrite)
{
    SCOPED_TRACE("Serial Stream To Serial Port Read and Write Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamToSerialPortReadWrite() ;
    }
}
