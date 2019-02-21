/******************************************************************************
 * @file MultiThreadUnitTests.cpp                                            *
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

#include "MultiThreadUnitTests.h"
#include "UnitTests.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace LibSerial;

MultiThreadUnitTests::MultiThreadUnitTests()
{
    // Empty
}

MultiThreadUnitTests::~MultiThreadUnitTests()
{
    // Empty
}

void
MultiThreadUnitTests::serialStream1ThreadLoop()
{
    size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds() ;
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        //
        // Write data to the serial port. It should be received by the other
        // thread.
        //
        serialStream1 << writeString1 << std::endl;
        serialStream1.DrainWriteBuffer() ;

        //
        // Wait for data to arrive from the other thread
        //
        if (serialStream1.IsDataAvailable())
        {
            alarm(5) ;   // Set a system alarm in case getline() blocks longer than 5 seconds.
            readString2.clear() ;
            getline(serialStream1, readString2) ;
            alarm(0) ;   // Deactivate the alarm.
            
            if(readString2 != writeString2)
            {
                const std::lock_guard<std::mutex> lock {mutex} ;
                failureRate++;
            } 
        }

        const std::lock_guard<std::mutex> lock {mutex} ;
        loopCount++;

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;
    }
}

void
MultiThreadUnitTests::serialStream2ThreadLoop()
{
    const size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds() ;
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        //
        // Write data to the serial port. It should be received by the other
        // thread.
        //
        serialStream2 << writeString2 << std::endl;
        serialStream2.DrainWriteBuffer() ;

        //
        // Wait for data to arrive from the other thread
        //
        if (serialStream2.IsDataAvailable())
        {
            alarm(5) ;   // Set a system alarm in case getline() blocks longer than 5 seconds.
            readString1.clear() ;
            getline(serialStream2, readString1) ;
            alarm(0) ;   // Deactivate the alarm.

            if(readString1 != writeString1)
            {
                const std::lock_guard<std::mutex> lock {mutex} ;
                failureRate++;
            } 
        } 

        const std::lock_guard<std::mutex> lock {mutex} ;
        loopCount++;

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;
    }
}

void
MultiThreadUnitTests::serialPort1ThreadLoop()
{
    const size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds() ;
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        //
        // Write data to the serial port. It should be received by the other
        // thread.
        //
        serialPort1.Write(writeString1 + '\n') ;
        serialPort1.DrainWriteBuffer() ;

        //
        // Wait for data to arrive from the other thread
        //
        if (serialPort1.IsDataAvailable())
        {
            try
            {
                readString2.clear() ;
                serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds) ;
            }
            catch (const std::exception& err)
            {
                std::cerr << err.what() << std::endl ;
            }

            if(readString2 != writeString2 + '\n')
            {
                const std::lock_guard<std::mutex> lock {mutex} ;
                failureRate++;
            }
        } 

        const std::lock_guard<std::mutex> lock {mutex} ;
        loopCount++;

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;
    }
}

void
MultiThreadUnitTests::serialPort2ThreadLoop()
{
    const size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds() ;
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        //
        // Write data to the serial port. It should be received by the other
        // thread.
        //
        serialPort2.Write(writeString2 + '\n') ;
        serialPort2.DrainWriteBuffer() ;

        //
        // Wait for data to arrive from the other thread
        //
        if (serialPort2.IsDataAvailable())
        {
            try
            {
                readString1.clear() ;
                serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds) ;
            }
            catch (const std::exception& err)
            {
                std::cerr << err.what() << std::endl ;
            }

            if(readString1 != writeString1 + '\n')
            {
                const std::lock_guard<std::mutex> lock {mutex} ;
                failureRate++;
            }
        }

        const std::lock_guard<std::mutex> lock {mutex} ;
        loopCount++;

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;
    }
}

void
MultiThreadUnitTests::testMultiThreadSerialStreamReadWrite()
{
    //
    // We must open the two serial ports before spawning the threads.
    // Otherwise, one thread may flush the serial port I/O buffers *after* the
    // other thread has already sent data. 
    //
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream1.SetBaudRate(BaudRate::BAUD_115200) ;
    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    serialStream2.Open(SERIAL_PORT_2) ;
    serialStream2.SetBaudRate(BaudRate::BAUD_115200) ;
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    std::thread serialStream1Thread(&MultiThreadUnitTests::serialStream1ThreadLoop, this) ;
    std::thread serialStream2Thread(&MultiThreadUnitTests::serialStream2ThreadLoop, this) ;

    serialStream1Thread.join() ;
    serialStream2Thread.join() ;

    serialStream1.Close() ;
    serialStream2.Close() ;
}

void
MultiThreadUnitTests::testMultiThreadSerialPortReadWrite()
{
    //
    // We must open the two serial ports before spawning the threads.
    // Otherwise, one thread may flush the serial port I/O buffers *after* the
    // other thread has already sent data. 
    //
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort1.SetBaudRate(BaudRate::BAUD_115200) ;
    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    serialPort2.Open(SERIAL_PORT_2) ;
    serialPort2.SetBaudRate(BaudRate::BAUD_115200) ;
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    std::thread serialPort1Thread(&MultiThreadUnitTests::serialPort1ThreadLoop, this) ;
    std::thread serialPort2Thread(&MultiThreadUnitTests::serialPort2ThreadLoop, this) ;

    serialPort1Thread.join() ;
    serialPort2Thread.join() ;

    serialPort1.Close() ;
    serialPort2.Close() ;
}

TEST_F(MultiThreadUnitTests, testMultiThreadSerialStreamReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Stream Communication.") ;

    failureRate = 0 ;
    loopCount = 0 ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testMultiThreadSerialStreamReadWrite() ;
    }

    const auto failRate = (
        100. * static_cast<double>(failureRate) /
        static_cast<double>(loopCount)
    ) ;
    
    // If the serial communication fail rate is greater than 0.0001% consider it a failed test.
    if (failRate > 0.0001)
    {
        std::cout << "\t     SerialStream Failure Rate = " << failRate << "%" << std::endl;
        ADD_FAILURE() ;
    }
}

TEST_F(MultiThreadUnitTests, testMultiThreadSerialPortReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Port Communication.") ;

    failureRate = 0;
    loopCount = 0;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testMultiThreadSerialPortReadWrite() ;
    }

    const auto failRate = (
        100. * static_cast<double>(failureRate) /
        static_cast<double>(loopCount)
    ) ;
    
    // If the serial communication fail rate is greater than 0.0001% consider it a failed test.
    if (failRate > 0.0001)
    {
        std::cout << "\t     SerialPort Failure Rate = " << failRate << "%" << std::endl;
        ADD_FAILURE() ;
    }
}
