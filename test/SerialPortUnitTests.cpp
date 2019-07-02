/******************************************************************************
 * @file SerialPortUnitTests.cpp                                              *
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

#include "SerialPortUnitTests.h"
#include "UnitTests.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace LibSerial;

SerialPortUnitTests::SerialPortUnitTests()
{
    // Empty
}

SerialPortUnitTests::~SerialPortUnitTests()
{
    // Empty
}

void
SerialPortUnitTests::testSerialPortConstructors()
{
    SerialPort serialPort3(SERIAL_PORT_1) ;
    SerialPort serialPort4(SERIAL_PORT_2,
                           BaudRate::BAUD_9600,
                           CharacterSize::CHAR_SIZE_7,
                           FlowControl::FLOW_CONTROL_HARDWARE,
                           Parity::PARITY_EVEN,
                           StopBits::STOP_BITS_2) ;

    ASSERT_TRUE(serialPort3.IsOpen()) ;
    ASSERT_TRUE(serialPort4.IsOpen()) ;
    
    ASSERT_EQ(serialPort4.GetBaudRate(),      BaudRate::BAUD_9600) ;
    ASSERT_EQ(serialPort4.GetCharacterSize(), CharacterSize::CHAR_SIZE_7) ;
    ASSERT_EQ(serialPort4.GetFlowControl(),   FlowControl::FLOW_CONTROL_HARDWARE) ;
    ASSERT_EQ(serialPort4.GetParity(),        Parity::PARITY_EVEN) ;
    ASSERT_EQ(serialPort4.GetStopBits(),      StopBits::STOP_BITS_2) ;
    
    serialPort3.Close() ;
    serialPort4.Close() ;

    ASSERT_FALSE(serialPort3.IsOpen()) ;
    ASSERT_FALSE(serialPort4.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortOpenClose()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    SerialPort serialPort3;
    SerialPort serialPort4;

    ASSERT_FALSE(serialPort3.IsOpen()) ;
    ASSERT_FALSE(serialPort4.IsOpen()) ;

    bool exclusiveUseTestPass = false;

    try
    {
        serialPort3.Open(SERIAL_PORT_1) ;
        serialPort4.Open(SERIAL_PORT_2) ;
    }
    catch (...)
    {
        exclusiveUseTestPass = true;
    }

    ASSERT_TRUE(exclusiveUseTestPass) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortDrainWriteBuffer()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.FlushIOBuffers() ;
    serialPort2.FlushIOBuffers() ;

    serialPort1.Write(writeString1) ;
    serialPort2.Write(writeString2) ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialPort1.IsDataAvailable()) ;
    ASSERT_TRUE(serialPort2.IsDataAvailable()) ;

    serialPort1.FlushIOBuffers() ;
    serialPort2.FlushIOBuffers() ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortFlushInputBuffer()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.FlushInputBuffer() ;
    serialPort2.FlushInputBuffer() ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    char writeByte = 'A';

    serialPort1.WriteByte(writeByte) ;
    serialPort2.WriteByte(writeByte) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialPort1.IsDataAvailable()) ;
    ASSERT_TRUE(serialPort2.IsDataAvailable()) ;

    serialPort1.FlushInputBuffer() ;
    serialPort2.FlushInputBuffer() ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortFlushOutputBuffer()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.FlushInputBuffer() ;
    serialPort2.FlushInputBuffer() ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    char writeByte = 'A';
    
    serialPort1.WriteByte(writeByte) ;
    serialPort1.FlushOutputBuffer() ;

    serialPort2.WriteByte(writeByte) ;
    serialPort2.FlushOutputBuffer() ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    serialPort1.Close() ;
    serialPort2.Close() ;
    
    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortFlushIOBuffers()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    char writeByte = 'a';

    serialPort1.WriteByte(writeByte) ;
    serialPort1.FlushIOBuffers() ;

    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    serialPort2.WriteByte(writeByte) ;
    serialPort2.FlushIOBuffers() ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;

    serialPort1.WriteByte(writeByte) ;
    serialPort2.WriteByte(writeByte) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;
    
    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialPort1.IsDataAvailable()) ;
    ASSERT_TRUE(serialPort2.IsDataAvailable()) ;

    serialPort1.FlushIOBuffers() ;
    serialPort2.FlushIOBuffers() ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortIsDataAvailableTest()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    char writeByte = 'a';
    char readByte = 'b';

    serialPort1.WriteByte(writeByte) ;
    serialPort1.DrainWriteBuffer() ;

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialPort2.IsDataAvailable()) ;

    serialPort2.ReadByte(readByte, 1) ;
    ASSERT_EQ(readByte, writeByte) ;

    serialPort2.WriteByte(writeByte) ;
    serialPort2.DrainWriteBuffer() ;

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialPort1.IsDataAvailable()) ;

    serialPort1.ReadByte(readByte, 1) ;
    ASSERT_EQ(readByte, writeByte) ;

    ASSERT_FALSE(serialPort1.IsDataAvailable()) ;
    ASSERT_FALSE(serialPort2.IsDataAvailable()) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortIsOpenTest()
{
    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;

    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetBaudRate()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    for(const auto baud_rate: baudRates)
    {
        serialPort1.SetBaudRate(baud_rate) ;
        serialPort2.SetBaudRate(baud_rate) ;

        const auto baudRate1 = serialPort1.GetBaudRate() ;
        const auto baudRate2 = serialPort1.GetBaudRate() ;

        ASSERT_EQ(baudRate1, baud_rate) ;
        ASSERT_EQ(baudRate2, baud_rate) ;
    }

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetCharacterSize()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    for(const auto char_size: characterSizes)
    {
        // @NOTE - Smaller CharSize values appear to be invalid on x86 Linux.
        if (char_size < CharacterSize::CHAR_SIZE_7)
        {
            continue;
        }

        serialPort1.SetCharacterSize(char_size) ;
        serialPort2.SetCharacterSize(char_size) ;

        const auto characterSize1 = serialPort1.GetCharacterSize() ;
        const auto characterSize2 = serialPort2.GetCharacterSize() ;

        ASSERT_EQ(characterSize1, char_size) ;
        ASSERT_EQ(characterSize2, char_size) ;
    }

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetFlowControl()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    for (const auto flow_control: flowControlTypes)
    {
        // @NOTE - FLOW_CONTROL_SOFT flow control appears to be invalid on x86 Linux.
        if (flow_control == FlowControl::FLOW_CONTROL_SOFTWARE)
        {
            continue;
        }

        serialPort1.SetFlowControl(flow_control) ;
        serialPort2.SetFlowControl(flow_control) ;

        const auto flowControl1 = serialPort1.GetFlowControl() ;
        const auto flowControl2 = serialPort2.GetFlowControl() ;

        ASSERT_EQ(flowControl1, flow_control) ;
        ASSERT_EQ(flowControl2, flow_control) ;
    }

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetParity()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    for (const auto parity: parityTypes)
    {
        serialPort1.SetParity(parity) ;
        serialPort2.SetParity(parity) ;

        const auto parity1 = serialPort1.GetParity() ;
        const auto parity2 = serialPort2.GetParity() ;

        ASSERT_EQ(parity1, parity) ;
        ASSERT_EQ(parity2, parity) ;
    }

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetStopBits()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    for (const auto stop_bits: stopBits)
    {
        serialPort1.SetStopBits(stop_bits) ;
        serialPort2.SetStopBits(stop_bits) ;

        const auto stopBits1 = serialPort1.GetStopBits() ;
        const auto stopBits2 = serialPort2.GetStopBits() ;

        ASSERT_EQ(stopBits1, stop_bits) ;
        ASSERT_EQ(stopBits2, stop_bits) ;
    }

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetVMin()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    ASSERT_TRUE(serialPort1.IsOpen()) ;

    for (short i = 0; i < 5; i++)
    {
        serialPort1.SetVMin(i) ;
        short vMin = serialPort1.GetVMin() ;
        ASSERT_EQ(vMin, i) ;
    }

    serialPort1.Close() ;
    ASSERT_FALSE(serialPort1.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetVTime()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    ASSERT_TRUE(serialPort1.IsOpen()) ;

    for (short i = 0; i < 5; i++)
    {
        serialPort1.SetVTime(i) ;
        short vTime = serialPort1.GetVTime() ;
        ASSERT_EQ(vTime, i) ;
    }

    serialPort1.Close() ;
    ASSERT_FALSE(serialPort1.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetDTR()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool dtrLine1 = false;
    bool dtrLine2 = false;

    dtrLine1 = serialPort1.GetDTR() ;
    dtrLine2 = serialPort2.GetDTR() ;

    ASSERT_TRUE(dtrLine1) ;
    ASSERT_TRUE(dtrLine2) ;

    serialPort1.SetDTR(false) ;
    serialPort2.SetDTR(false) ;

    dtrLine1 = serialPort1.GetDTR() ;
    dtrLine2 = serialPort2.GetDTR() ;

    ASSERT_FALSE(dtrLine1) ;
    ASSERT_FALSE(dtrLine2) ;

    serialPort1.SetDTR(true) ;
    serialPort2.SetDTR(true) ;

    dtrLine1 = serialPort1.GetDTR() ;
    dtrLine2 = serialPort2.GetDTR() ;

    ASSERT_TRUE(dtrLine1) ;
    ASSERT_TRUE(dtrLine2) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetGetRTS()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool rtsLine1 = false;
    bool rtsLine2 = false;

    rtsLine1 = serialPort1.GetRTS() ;
    rtsLine2 = serialPort2.GetRTS() ;
    
    ASSERT_TRUE(rtsLine1) ;
    ASSERT_TRUE(rtsLine2) ;

    serialPort1.SetRTS(false) ;
    serialPort2.SetRTS(false) ;
    
    rtsLine1 = serialPort1.GetRTS() ;
    rtsLine2 = serialPort2.GetRTS() ;
    
    ASSERT_FALSE(rtsLine1) ;
    ASSERT_FALSE(rtsLine2) ;

    serialPort1.SetRTS(true) ;
    serialPort2.SetRTS(true) ;

    rtsLine1 = serialPort1.GetRTS() ;
    rtsLine2 = serialPort2.GetRTS() ;

    ASSERT_TRUE(rtsLine1) ;
    ASSERT_TRUE(rtsLine2) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetRTSGetCTS()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool ctsLine1 = false;
    bool ctsLine2 = false;

    ctsLine1 = serialPort1.GetCTS() ;
    ctsLine2 = serialPort2.GetCTS() ;

    ASSERT_TRUE(ctsLine1) ;
    ASSERT_TRUE(ctsLine2) ;

    serialPort1.SetRTS(false) ;
    serialPort2.SetRTS(false) ;

    ctsLine1 = serialPort1.GetCTS() ;
    ctsLine2 = serialPort2.GetCTS() ;

    ASSERT_FALSE(ctsLine1) ;
    ASSERT_FALSE(ctsLine2) ;

    serialPort1.SetRTS(true) ;
    serialPort2.SetRTS(true) ;

    ctsLine1 = serialPort1.GetCTS() ;
    ctsLine2 = serialPort2.GetCTS() ;

    ASSERT_TRUE(ctsLine1) ;
    ASSERT_TRUE(ctsLine2) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortSetDTRGetDSR()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool dsrStatus1 = false;
    bool dsrStatus2 = false;

    dsrStatus1 = serialPort1.GetDSR() ;
    dsrStatus2 = serialPort1.GetDSR() ;

    ASSERT_TRUE(dsrStatus1) ;
    ASSERT_TRUE(dsrStatus2) ;

    serialPort1.SetDTR(false) ;
    serialPort2.SetDTR(false) ;

    dsrStatus1 = serialPort1.GetDSR() ;
    dsrStatus2 = serialPort2.GetDSR() ;

    ASSERT_FALSE(dsrStatus1) ;
    ASSERT_FALSE(dsrStatus2) ;

    serialPort1.SetDTR(true) ;
    serialPort2.SetDTR(true) ;

    dsrStatus1 = serialPort1.GetDSR() ;
    dsrStatus2 = serialPort2.GetDSR() ;

    ASSERT_TRUE(dsrStatus1) ;
    ASSERT_TRUE(dsrStatus2) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortGetFileDescriptor()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    int fileDescriptor1 = serialPort1.GetFileDescriptor() ;
    int fileDescriptor2 = serialPort2.GetFileDescriptor() ;

    ASSERT_GT(fileDescriptor1, 0) ;
    ASSERT_GT(fileDescriptor2, 0) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortGetNumberOfBytesAvailable()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    int bytesAvailable1 = 0;
    int bytesAvailable2 = 0;

    bytesAvailable1 = serialPort1.GetNumberOfBytesAvailable() ;
    bytesAvailable2 = serialPort2.GetNumberOfBytesAvailable() ;

    serialPort1.WriteByte(writeByte1) ;
    serialPort2.WriteByte(writeByte2) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    usleep(readBufferDelay) ;

    bytesAvailable1 = serialPort1.GetNumberOfBytesAvailable() ;
    bytesAvailable2 = serialPort2.GetNumberOfBytesAvailable() ;

    ASSERT_EQ(bytesAvailable1, 1) ;
    ASSERT_EQ(bytesAvailable2, 1) ;

    serialPort1.ReadByte(readByte2, 1) ;
    serialPort2.ReadByte(readByte1, 1) ;

    bytesAvailable1 = serialPort1.GetNumberOfBytesAvailable() ;
    bytesAvailable2 = serialPort2.GetNumberOfBytesAvailable() ;

    ASSERT_EQ(bytesAvailable1, 0) ;
    ASSERT_EQ(bytesAvailable2, 0) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

#ifdef __linux__
void
SerialPortUnitTests::testSerialPortGetAvailableSerialPorts()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    const auto portCount1 = serialPort1.GetAvailableSerialPorts() ;
    const auto portCount2 = serialPort2.GetAvailableSerialPorts() ;

    ASSERT_GE(portCount1.size(), 2UL) ;
    ASSERT_GE(portCount2.size(), 2UL) ;
    ASSERT_EQ(portCount1, portCount2) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}
#endif

void
SerialPortUnitTests::testSerialPortReadDataBufferWriteDataBuffer()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    constexpr size_t data_count = 75;

    DataBuffer writeVector1;
    DataBuffer writeVector2;

    // Test using ASCII characters.
    for (unsigned char i = 0; i < data_count; i++)
    {
        writeVector1.push_back((48 + i)) ;
        writeVector2.push_back((122 - i)) ;
    }

    serialPort1.Write(writeVector1) ;
    serialPort2.Write(writeVector2) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    DataBuffer readVector1;
    DataBuffer readVector2;

    serialPort1.Read(readVector2, data_count, timeOutMilliseconds) ;
    serialPort2.Read(readVector1, data_count, timeOutMilliseconds) ;

    ASSERT_EQ(readVector1, writeVector1) ;
    ASSERT_EQ(readVector2, writeVector2) ;

    readVector1.clear() ;
    readVector2.clear() ;

    //
    // Read data on serialPort2 for a certain period of time by setting the
    // second argument of SerialPort::Read() to 0. Make sure that the
    // time duration is correct.
    //
    using namespace std::literals::chrono_literals ;
    constexpr auto TEST_READ_DURATION = 75ms ;
    const auto start_time = std::chrono::high_resolution_clock::now() ;
    bool timeOutTestPass = false;
    try
    {
        serialPort1.Write(writeVector1) ;
        serialPort1.DrainWriteBuffer() ;
        serialPort2.Read(readVector1, 0, TEST_READ_DURATION.count()) ;
    }
    catch(const ReadTimeout&)
    {
        //
        // Make sure that the timeout period was close to what we specified
        // within a small margin.
        //
        constexpr auto TIMEOUT_DURATION_MARGIN = 30ms ;
        const auto elapsed_time =(
            std::chrono::high_resolution_clock::now() - start_time) ;
        if (elapsed_time - TEST_READ_DURATION < TIMEOUT_DURATION_MARGIN) {
            timeOutTestPass = true;
        }
    }

    ASSERT_TRUE(timeOutTestPass) ;
    ASSERT_EQ(readVector1, writeVector1) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortReadStringWriteString()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    bool timeOutTestPass = false;

    serialPort1.Write(writeString1) ;
    serialPort2.Write(writeString2) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    serialPort1.Read(readString2, writeString2.size(), timeOutMilliseconds) ;
    serialPort2.Read(readString1, writeString1.size(), timeOutMilliseconds) ;

    ASSERT_EQ(readString1, writeString1) ;
    ASSERT_EQ(readString2, writeString2) ;

    timeOutTestPass = false;

    readString1.clear() ;
    readString2.clear() ;

    try
    {
        serialPort1.Write(writeString1) ;

        serialPort1.DrainWriteBuffer() ;

        serialPort2.Read(readString1, 0, 75) ;
    }
    catch (...)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass) ;
    ASSERT_EQ(readString1, writeString1) ;
    
    serialPort1.Close() ;
    serialPort2.Close() ;
    
    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortReadByteWriteByte()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    bool timeOutTestPass = false;

    char writeByte1 = 'a';
    unsigned char writeByte2 = 'A';

    char readByte1  = 'b';
    unsigned char readByte2  = 'B';

    serialPort1.WriteByte(writeByte1) ;
    serialPort2.WriteByte(writeByte2) ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    serialPort1.ReadByte(readByte2, timeOutMilliseconds) ;
    serialPort2.ReadByte(readByte1, timeOutMilliseconds) ;

    ASSERT_EQ(readByte1, writeByte1) ;
    ASSERT_EQ(readByte2, writeByte2) ;

    try
    {
        serialPort1.ReadByte(readByte1, 1) ;
        serialPort2.ReadByte(readByte2, 1) ;
    }
    catch (...)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

void
SerialPortUnitTests::testSerialPortReadLineWriteString()
{
    serialPort1.Open(SERIAL_PORT_1) ;
    serialPort2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialPort1.IsOpen()) ;
    ASSERT_TRUE(serialPort2.IsOpen()) ;

    bool timeOutTestPass = false;

    serialPort1.Write(writeString1 + '\n') ;
    serialPort2.Write(writeString2 + '\n') ;

    serialPort1.DrainWriteBuffer() ;
    serialPort2.DrainWriteBuffer() ;

    serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds) ;
    serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds) ;

    ASSERT_EQ(readString1, writeString1 + '\n') ;
    ASSERT_EQ(readString2, writeString2 + '\n') ;

    try
    {
        serialPort1.ReadLine(readString2, '\n', 1) ;
        serialPort2.ReadLine(readString1, '\n', 1) ;
    }
    catch (...)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass) ;

    serialPort1.Close() ;
    serialPort2.Close() ;

    ASSERT_FALSE(serialPort1.IsOpen()) ;
    ASSERT_FALSE(serialPort2.IsOpen()) ;
}

TEST_F(SerialPortUnitTests, testSerialPortConstructors)
{
    SCOPED_TRACE("Serial Port Constructors Tests") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortConstructors() ;
    }  
}

TEST_F(SerialPortUnitTests, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open() and Close() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortOpenClose() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortDrainWriteBuffer)
{
    SCOPED_TRACE("Serial Port DrainWriteBuffer() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortDrainWriteBuffer() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortFlushInputBuffer)
{
    SCOPED_TRACE("Serial Port FlushInputBuffer() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortFlushInputBuffer() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Port FlushOutputBuffer() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortFlushOutputBuffer() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortFlushIOBuffers)
{
    SCOPED_TRACE("Serial Port FlushIOBuffers() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortFlushIOBuffers() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port IsDataAvailable() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortIsDataAvailableTest() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortIsOpenTest)
{
    SCOPED_TRACE("Serial Port IsOpen() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortIsOpenTest() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port SetBaudRate() and GetBaudRate() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetBaudRate() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port SetCharacterSize() and GetCharacterSize() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetCharacterSize() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port SetParityType() and GetParityType() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetParity() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port SetFlowControl() and GetFlowControl() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetFlowControl() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port SetStopBits() and GetStopBits() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetStopBits() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetVMin() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetVTime() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDTR() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetDTR() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetRTS() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetGetRTS() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetRTSGetCTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetCTS() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetRTSGetCTS() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortSetDTRGetDSR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDSR() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortSetDTRGetDSR() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortGetFileDescriptor)
{
    SCOPED_TRACE("Serial Port GetFileDescriptor() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortGetFileDescriptor() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortGetNumberOfBytesAvailable)
{
    SCOPED_TRACE("Serial Port GetNumberOfBytesAvailable() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortGetNumberOfBytesAvailable() ;
    }
}

#ifdef __linux__
TEST_F(SerialPortUnitTests, testSerialPortGetAvailableSerialPorts)
{
    SCOPED_TRACE("Serial Port GetAvailableSerialPorts() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortGetAvailableSerialPorts() ;
    }
}
#endif

TEST_F(SerialPortUnitTests, testSerialPortReadDataBufferWriteDataBuffer)
{
    SCOPED_TRACE("Serial Port Read(DataBuffer) and Write(DataBuffer) Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortReadDataBufferWriteDataBuffer() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortReadStringWriteString)
{
    SCOPED_TRACE("Serial Port Read(string) and Write(string) Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortReadStringWriteString() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortReadByteWriteByte)
{
    SCOPED_TRACE("Serial Port ReadByte() and WriteByte() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortReadByteWriteByte() ;
    }
}

TEST_F(SerialPortUnitTests, testSerialPortReadLineWriteString)
{
    SCOPED_TRACE("Serial Port ReadLine() and Write(string) Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialPortReadLineWriteString() ;
    }
}
