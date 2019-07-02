/******************************************************************************
 * @file SerialStreamUnitTests.cpp                                            *
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

#include "SerialStreamUnitTests.h"
#include "UnitTests.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace LibSerial;

SerialStreamUnitTests::SerialStreamUnitTests()
{
    // Empty
}

SerialStreamUnitTests::~SerialStreamUnitTests()
{
    // Empty
}

void
SerialStreamUnitTests::testSerialStreamConstructors()
{
    SerialStream serialStream3(SERIAL_PORT_1) ;
    SerialStream serialStream4(SERIAL_PORT_2,
                               BaudRate::BAUD_9600,
                               CharacterSize::CHAR_SIZE_7,
                               FlowControl::FLOW_CONTROL_HARDWARE,
                               Parity::PARITY_EVEN,
                               StopBits::STOP_BITS_2) ;

    ASSERT_TRUE(serialStream3.IsOpen()) ;
    ASSERT_TRUE(serialStream4.IsOpen()) ;
    
    ASSERT_EQ(serialStream4.GetBaudRate(),      BaudRate::BAUD_9600) ;
    ASSERT_EQ(serialStream4.GetCharacterSize(), CharacterSize::CHAR_SIZE_7) ;
    ASSERT_EQ(serialStream4.GetFlowControl(),   FlowControl::FLOW_CONTROL_HARDWARE) ;
    ASSERT_EQ(serialStream4.GetParity(),        Parity::PARITY_EVEN) ;
    ASSERT_EQ(serialStream4.GetStopBits(),      StopBits::STOP_BITS_2) ;
    
    serialStream3.Close() ;
    serialStream4.Close() ;

    ASSERT_FALSE(serialStream3.IsOpen()) ;
    ASSERT_FALSE(serialStream4.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamOpenClose()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    ASSERT_TRUE(serialStream1.good()) ;
    ASSERT_TRUE(serialStream2.good()) ;

    SerialStream serialStream3;
    SerialStream serialStream4;

    ASSERT_FALSE(serialStream3.IsOpen()) ;
    ASSERT_FALSE(serialStream4.IsOpen()) ;

    ASSERT_FALSE(serialStream3.good()) ;
    ASSERT_FALSE(serialStream4.good()) ;

    bool exclusiveUseTestPass = false;

    try
    {
        serialStream3.Open(SERIAL_PORT_1) ;
        serialStream4.Open(SERIAL_PORT_2) ;
    }
    catch (...)
    {
        exclusiveUseTestPass = true;
    }

    ASSERT_TRUE(exclusiveUseTestPass) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamDrainWriteBuffer()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.FlushIOBuffers() ;
    serialStream2.FlushIOBuffers() ;

    serialStream1 << writeString1 << std::endl;
    serialStream2 << writeString2 << std::endl;

    serialStream1.DrainWriteBuffer() ;
    serialStream2.DrainWriteBuffer() ;

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialStream1.IsDataAvailable()) ;
    ASSERT_TRUE(serialStream2.IsDataAvailable()) ;

    serialStream1.FlushIOBuffers() ;
    serialStream2.FlushIOBuffers() ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamFlushInputBuffer()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.FlushInputBuffer() ;
    serialStream2.FlushInputBuffer() ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    char writeByte = 'A';

    serialStream1.write(&writeByte, 1) ;
    serialStream2.write(&writeByte, 1) ;

    serialStream1.DrainWriteBuffer() ;
    serialStream2.DrainWriteBuffer() ;

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialStream1.IsDataAvailable()) ;
    ASSERT_TRUE(serialStream2.IsDataAvailable()) ;

    serialStream1.FlushInputBuffer() ;
    serialStream2.FlushInputBuffer() ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamFlushOutputBuffer()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.FlushInputBuffer() ;
    serialStream2.FlushInputBuffer() ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    char writeByte = 'A';

    serialStream1.write(&writeByte, 1) ;
    serialStream1.FlushOutputBuffer() ;

    serialStream2.write(&writeByte, 1) ;
    serialStream2.FlushOutputBuffer() ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamFlushIOBuffers()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    char writeByte = 'a';

    serialStream1.write(&writeByte, 1) ;
    serialStream1.FlushIOBuffers() ;

    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    serialStream2.write(&writeByte, 1) ;
    serialStream2.FlushIOBuffers() ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;

    serialStream1.write(&writeByte, 1) ;
    serialStream2.write(&writeByte, 1) ;

    serialStream1.DrainWriteBuffer() ;
    serialStream2.DrainWriteBuffer() ;
    
    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialStream1.IsDataAvailable()) ;
    ASSERT_TRUE(serialStream2.IsDataAvailable()) ;

    serialStream1.FlushIOBuffers() ;
    serialStream2.FlushIOBuffers() ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    serialStream1.Close() ;
    serialStream2.Close() ;
    
    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamIsDataAvailableTest()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    char writeByte = 'a';
    char readByte = 'b';

    serialStream1.write(&writeByte, 1) ;
    serialStream1.DrainWriteBuffer() ;

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialStream2.IsDataAvailable()) ;

    serialStream2.read(&readByte, 1) ;
    ASSERT_EQ(readByte, writeByte) ;

    serialStream2.write(&writeByte, 1) ;
    serialStream2.DrainWriteBuffer() ;

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay) ;

    ASSERT_TRUE(serialStream1.IsDataAvailable()) ;

    serialStream1.read(&readByte, 1) ;
    ASSERT_EQ(readByte, writeByte) ;

    ASSERT_FALSE(serialStream1.IsDataAvailable()) ;
    ASSERT_FALSE(serialStream2.IsDataAvailable()) ;

    serialStream1.Close() ;
    serialStream2.Close() ;
    
    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamIsOpenTest()
{
    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;

    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetBaudRate()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    for (const auto baud_rate: baudRates)
    {
        serialStream1.SetBaudRate(baud_rate) ;
        serialStream2.SetBaudRate(baud_rate) ;

        const auto baudRate1 = serialStream1.GetBaudRate() ;
        const auto baudRate2 = serialStream2.GetBaudRate() ;

        ASSERT_EQ(baudRate1, baud_rate) ;
        ASSERT_EQ(baudRate2, baud_rate) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetCharacterSize()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    for (const auto char_size: characterSizes)
    {
        // @NOTE - Smaller Character Size values do not work in Linux.
        if (char_size < CharacterSize::CHAR_SIZE_7)
        {
            continue;
        }
        
        serialStream1.SetCharacterSize(char_size) ;
        serialStream2.SetCharacterSize(char_size) ;

        const auto characterSize1 = serialStream1.GetCharacterSize() ;
        const auto characterSize2 = serialStream2.GetCharacterSize() ;

        ASSERT_EQ(characterSize1, char_size) ;
        ASSERT_EQ(characterSize2, char_size) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetFlowControl()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    for (const auto flow_control: flowControlTypes)
    {
        serialStream1.SetFlowControl(flow_control) ;
        serialStream1.SetFlowControl(flow_control) ;

        const auto flowControl1 = serialStream1.GetFlowControl() ;
        const auto flowControl2 = serialStream1.GetFlowControl() ;

        ASSERT_EQ(flowControl1, flow_control) ;
        ASSERT_EQ(flowControl2, flow_control) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetParity()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    for (const auto parity: parityTypes)
    {
        serialStream1.SetParity(parity) ;
        serialStream2.SetParity(parity) ;

        const auto parity1 = serialStream1.GetParity() ;
        const auto parity2 = serialStream2.GetParity() ;

        ASSERT_EQ(parity1, parity) ;
        ASSERT_EQ(parity2, parity) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetStopBits()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    for (const auto stop_bits: stopBits)
    {
        serialStream1.SetStopBits(stop_bits) ;
        serialStream2.SetStopBits(stop_bits) ;

        const auto numberOfStopBits1 = serialStream1.GetStopBits() ;
        const auto numberOfStopBits2 = serialStream2.GetStopBits() ;

        ASSERT_EQ(numberOfStopBits1, stop_bits) ;
        ASSERT_EQ(numberOfStopBits2, stop_bits) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetVMin()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    short vMin1 = 0;
    short vMin2 = 0;

    for (short i = 0; i < 5; i++)
    {
        serialStream1.SetVMin(i) ;
        serialStream2.SetVMin(i) ;

        vMin1 = serialStream1.GetVMin() ;
        vMin2 = serialStream2.GetVMin() ;

        ASSERT_EQ(vMin1, i) ;
        ASSERT_EQ(vMin2, i) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetVTime()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    short vTime1 = 0;
    short vTime2 = 0;

    for (short i = 0; i < 5; i++)
    {
        serialStream1.SetVTime(i) ;
        serialStream2.SetVTime(i) ;

        vTime1 = serialStream1.GetVTime() ;
        vTime2 = serialStream2.GetVTime() ;

        ASSERT_EQ(vTime1, i) ;
        ASSERT_EQ(vTime2, i) ;
    }

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetDTR()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool dtrLine1 = false;
    bool dtrLine2 = false;

    dtrLine1 = serialStream1.GetDTR() ;
    dtrLine2 = serialStream2.GetDTR() ;

    ASSERT_TRUE(dtrLine1) ;
    ASSERT_TRUE(dtrLine2) ;

    serialStream1.SetDTR(true) ;
    serialStream2.SetDTR(true) ;

    dtrLine1 = serialStream1.GetDTR() ;
    dtrLine2 = serialStream2.GetDTR() ;

    ASSERT_TRUE(dtrLine1) ;
    ASSERT_TRUE(dtrLine2) ;

    serialStream1.SetDTR(false) ;
    serialStream2.SetDTR(false) ;

    dtrLine1 = serialStream1.GetDTR() ;
    dtrLine2 = serialStream2.GetDTR() ;

    ASSERT_FALSE(dtrLine1) ;
    ASSERT_FALSE(dtrLine2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetGetRTS()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool rtsLine1 = false;
    bool rtsLine2 = false;

    rtsLine1 = serialStream1.GetRTS() ;
    rtsLine2 = serialStream2.GetRTS() ;
    
    ASSERT_TRUE(rtsLine1) ;
    ASSERT_TRUE(rtsLine2) ;

    serialStream1.SetRTS(false) ;
    serialStream2.SetRTS(false) ;
    
    rtsLine1 = serialStream1.GetRTS() ;
    rtsLine2 = serialStream2.GetRTS() ;
    
    ASSERT_FALSE(rtsLine1) ;
    ASSERT_FALSE(rtsLine2) ;

    serialStream1.SetRTS(true) ;
    serialStream2.SetRTS(true) ;

    rtsLine1 = serialStream1.GetRTS() ;
    rtsLine2 = serialStream2.GetRTS() ;

    ASSERT_TRUE(rtsLine1) ;
    ASSERT_TRUE(rtsLine2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetRTSGetCTS()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool ctsLine1 = false;
    bool ctsLine2 = false;

    ctsLine1 = serialStream1.GetCTS() ;
    ctsLine2 = serialStream2.GetCTS() ;

    ASSERT_TRUE(ctsLine1) ;
    ASSERT_TRUE(ctsLine2) ;

    serialStream1.SetRTS(false) ;
    serialStream2.SetRTS(false) ;

    ctsLine1 = serialStream1.GetCTS() ;
    ctsLine2 = serialStream2.GetCTS() ;

    ASSERT_FALSE(ctsLine1) ;
    ASSERT_FALSE(ctsLine2) ;

    serialStream1.SetRTS(true) ;
    serialStream2.SetRTS(true) ;

    ctsLine1 = serialStream1.GetCTS() ;
    ctsLine2 = serialStream2.GetCTS() ;

    ASSERT_TRUE(ctsLine1) ;
    ASSERT_TRUE(ctsLine2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamSetDTRGetDSR()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE) ;

    bool dsrStatus1 = false;
    bool dsrStatus2 = false;

    dsrStatus1 = serialStream1.GetDSR() ;
    dsrStatus2 = serialStream2.GetDSR() ;

    ASSERT_TRUE(dsrStatus1) ;
    ASSERT_TRUE(dsrStatus2) ;

    serialStream1.SetDTR(false) ;
    serialStream2.SetDTR(false) ;

    dsrStatus1 = serialStream1.GetDSR() ;
    dsrStatus2 = serialStream2.GetDSR() ;

    ASSERT_FALSE(dsrStatus1) ;
    ASSERT_FALSE(dsrStatus2) ;

    serialStream1.SetDTR(true) ;
    serialStream2.SetDTR(true) ;

    dsrStatus1 = serialStream1.GetDSR() ;
    dsrStatus2 = serialStream2.GetDSR() ;

    ASSERT_TRUE(dsrStatus1) ;
    ASSERT_TRUE(dsrStatus2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamGetFileDescriptor()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    int fileDescriptor1 = serialStream1.GetFileDescriptor() ;
    int fileDescriptor2 = serialStream2.GetFileDescriptor() ;

    ASSERT_GT(fileDescriptor1, 0) ;
    ASSERT_GT(fileDescriptor2, 0) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamGetNumberOfBytesAvailable()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    int bytesAvailable1 = 0;
    int bytesAvailable2 = 0;

    serialStream1.write(&writeByte1, 1) ;
    serialStream2.write(&writeByte2, 1) ;

    serialStream1.DrainWriteBuffer() ;
    serialStream2.DrainWriteBuffer() ;

    usleep(readBufferDelay) ;

    bytesAvailable1 = serialStream1.GetNumberOfBytesAvailable() ;
    bytesAvailable2 = serialStream2.GetNumberOfBytesAvailable() ;

    ASSERT_EQ(bytesAvailable1, 1) ;
    ASSERT_EQ(bytesAvailable2, 1) ;

    serialStream1.read(&readByte2, 1) ;
    serialStream2.read(&readByte1, 1) ;

    bytesAvailable1 = serialStream1.GetNumberOfBytesAvailable() ;
    bytesAvailable2 = serialStream2.GetNumberOfBytesAvailable() ;

    ASSERT_EQ(bytesAvailable1, 0) ;
    ASSERT_EQ(bytesAvailable2, 0) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

#ifdef __linux__
void
SerialStreamUnitTests::testSerialStreamGetAvailableSerialPorts()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    const auto portCount1 = serialStream1.GetAvailableSerialPorts() ;
    const auto portCount2 = serialStream2.GetAvailableSerialPorts() ;

    ASSERT_GE(portCount1.size(), 2UL) ;
    ASSERT_GE(portCount2.size(), 2UL) ;
    ASSERT_EQ(portCount1, portCount2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}
#endif

void
SerialStreamUnitTests::testSerialStreamReadByteWriteByte()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    serialStream1.write(&writeByte1, 1) ;
    serialStream2.write(&writeByte2, 1) ;

    serialStream1.read(&readByte2, 1) ;
    serialStream2.read(&readByte1, 1) ;

    ASSERT_EQ(readByte1, writeByte1) ;
    ASSERT_EQ(readByte2, writeByte2) ;

    serialStream1 << writeByte1;
    serialStream2 << writeByte2;

    serialStream1.read(&readByte2, 1) ;
    serialStream2.read(&readByte1, 1) ;

    ASSERT_EQ(readByte1, writeByte1) ;
    ASSERT_EQ(readByte2, writeByte2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamGetLineWriteString()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    serialStream1 << writeString1 << std::endl;
    serialStream2 << writeString2 << std::endl;

    getline(serialStream1, readString2) ;
    getline(serialStream2, readString1) ;

    ASSERT_EQ(readString1, writeString1) ;
    ASSERT_EQ(readString2, writeString2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}

void
SerialStreamUnitTests::testSerialStreamGetWriteByte()
{
    serialStream1.Open(SERIAL_PORT_1) ;
    serialStream2.Open(SERIAL_PORT_2) ;

    ASSERT_TRUE(serialStream1.IsOpen()) ;
    ASSERT_TRUE(serialStream2.IsOpen()) ;

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    serialStream1.write(&writeByte1, 1) ;
    serialStream2.write(&writeByte2, 1) ;

    serialStream1.get(readByte2) ;
    serialStream2.get(readByte1) ;

    ASSERT_EQ(readByte1, writeByte1) ;
    ASSERT_EQ(readByte2, writeByte2) ;

    serialStream1 << writeByte1;
    serialStream2 << writeByte2;

    serialStream1.get(readByte2) ;
    serialStream2.get(readByte1) ;

    ASSERT_EQ(readByte1, writeByte1) ;
    ASSERT_EQ(readByte2, writeByte2) ;

    serialStream1.Close() ;
    serialStream2.Close() ;

    ASSERT_FALSE(serialStream1.IsOpen()) ;
    ASSERT_FALSE(serialStream2.IsOpen()) ;
}


TEST_F(SerialStreamUnitTests, testSerialStreamConstructors)
{
    SCOPED_TRACE("Serial Stream Constructor Tests") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamConstructors() ;
    }  
}

TEST_F(SerialStreamUnitTests, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open() and Close() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamOpenClose() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamDrainWriteBuffer)
{
    SCOPED_TRACE("Serial Stream DrainWriteBuffer() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamDrainWriteBuffer() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamFlushInputBuffer)
{
    SCOPED_TRACE("Serial Stream FlushInputBuffer() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamFlushInputBuffer() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Stream FlushOutputBuffer() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamFlushOutputBuffer() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamFlushIOBuffers)
{
    SCOPED_TRACE("Serial Stream FlushIOBuffers() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamFlushIOBuffers() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Stream IsDataAvailable() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamIsDataAvailableTest() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamIsOpenTest)
{
    SCOPED_TRACE("Serial Stream IsOpen() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamIsOpenTest() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream SetBaudRate() and GetBaudRate() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetBaudRate() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream SetCharacterSize() and GetCharacterSize() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetCharacterSize() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream SetParityType() and GetParityType() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetParity() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream SetFlowControl() and GetFlowControl() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetFlowControl() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream SetStopBits() and GetStopBits() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetStopBits() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Stream SetVMin() and GetVMin() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetVMin() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Stream SetVTime() and GetVTime() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetVTime() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetDTR)
{
    SCOPED_TRACE("Serial Stream SetDTR() and GetDTR() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetDTR() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetGetRTS)
{
    SCOPED_TRACE("Serial Stream SetRTS() and GetRTS() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetGetRTS() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetRTSGetCTS)
{
    SCOPED_TRACE("Serial Stream SetRTS() and GetCTS() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetRTSGetCTS() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamSetDTRGetDSR)
{
    SCOPED_TRACE("Serial Stream SetDTR() and GetDSR() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamSetDTRGetDSR() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamGetFileDescriptor)
{
    SCOPED_TRACE("Serial Stream GetFileDescriptor() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamGetFileDescriptor() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamGetNumberOfBytesAvailable)
{
    SCOPED_TRACE("Serial Stream GetNumberOfBytesAvailable() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamGetNumberOfBytesAvailable() ;
    }
}

#ifdef __linux__
TEST_F(SerialStreamUnitTests, testSerialStreamGetAvailableSerialPorts)
{
    SCOPED_TRACE("Serial Stream GetAvailableSerialPorts() Test") ;
    
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamGetAvailableSerialPorts() ;
    }
}
#endif

TEST_F(SerialStreamUnitTests, testSerialStreamReadByteWriteByte)
{
    SCOPED_TRACE("Serial Stream Read() and WriteByte() Test") ;

    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamReadByteWriteByte() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamGetLineWriteString)
{
    SCOPED_TRACE("Serial Stream GetLine() and Write(string) Test") ;
        
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamGetLineWriteString() ;
    }
}

TEST_F(SerialStreamUnitTests, testSerialStreamGetWriteByte)
{
    SCOPED_TRACE("Serial Stream Get() and WriteByte() Test") ;
        
    for (size_t i = 0; i < TEST_ITERATIONS; i++)
    {
        testSerialStreamGetWriteByte() ;
    }
}
