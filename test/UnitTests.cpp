/******************************************************************************
 *   @file LibSerialTest.cpp                                                  *
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


#include <chrono>
#include <iostream>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

#include "UnitTests.h"

using namespace LibSerial;

LibSerialTest::LibSerialTest()
    : numberOfTestIterations(10)
    , mutex{}
    , entryTime(0)
    , currentTime(0)
    , elapsedTime(0)
    , failureRate(0)
    , loopCount(0)
    , timeOutMilliseconds(250)   // 250ms timeout
    , readBufferDelay(20000)     // 20ms delay
    , baudRates{}
    , characterSizes{}
    , flowControlTypes{}
    , parityTypes{}
    , stopBits{}
    , serialStream1{}
    , serialStream2{}
    , serialPort1{}
    , serialPort2{}
    , readString1("")
    , readString2("")
    , writeString1("Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)")
    , writeString2("The secret of the man who is universally interesting is that he is universally interested. - William Dean Howells")
{
    /* Empty */
}


LibSerialTest::~LibSerialTest()
{
    /* Empty */
}

void
LibSerialTest::SetUp()
{
    // Any additional setup steps can be performed here.
}

size_t
LibSerialTest::getTimeInMilliSeconds()
{
    std::chrono::high_resolution_clock::duration timeNow = 
        std::chrono::high_resolution_clock::now().time_since_epoch();

    return std::chrono::duration_cast<std::chrono::milliseconds>(timeNow).count();
}

size_t
LibSerialTest::getTimeInMicroSeconds()
{
    std::chrono::high_resolution_clock::duration timeNow = 
        std::chrono::high_resolution_clock::now().time_since_epoch();

    return std::chrono::duration_cast<std::chrono::microseconds>(timeNow).count();
}


//---------------------- Serial Stream Unit Tests -----------------------//

void
LibSerialTest::testSerialStreamConstructors()
{
    SerialStream serialStream3(TEST_SERIAL_PORT_1);
    SerialStream serialStream4(TEST_SERIAL_PORT_2,
                               BaudRate::BAUD_9600,
                               CharacterSize::CHAR_SIZE_7,
                               FlowControl::FLOW_CONTROL_HARDWARE,
                               Parity::PARITY_EVEN,
                               StopBits::STOP_BITS_2);

    ASSERT_TRUE(serialStream3.IsOpen());
    ASSERT_TRUE(serialStream4.IsOpen());
    
    ASSERT_EQ(serialStream4.GetBaudRate(),      BaudRate::BAUD_9600);
    ASSERT_EQ(serialStream4.GetCharacterSize(), CharacterSize::CHAR_SIZE_7);
    ASSERT_EQ(serialStream4.GetFlowControl(),   FlowControl::FLOW_CONTROL_HARDWARE);
    ASSERT_EQ(serialStream4.GetParity(),        Parity::PARITY_EVEN);
    ASSERT_EQ(serialStream4.GetStopBits(),      StopBits::STOP_BITS_2);
    
    serialStream3.Close();
    serialStream4.Close();

    ASSERT_FALSE(serialStream3.IsOpen());
    ASSERT_FALSE(serialStream4.IsOpen());
}

void
LibSerialTest::testSerialStreamOpenClose()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    SerialStream serialStream3;
    SerialStream serialStream4;

    ASSERT_FALSE(serialStream3.IsOpen());
    ASSERT_FALSE(serialStream4.IsOpen());

    bool exclusiveUseTestPass = false;

    try
    {
        serialStream3.Open(TEST_SERIAL_PORT_1);
        serialStream4.Open(TEST_SERIAL_PORT_2);
    }
    catch (...)
    {
        exclusiveUseTestPass = true;
    }

    ASSERT_TRUE(exclusiveUseTestPass);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamDrainWriteBuffer()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.FlushIOBuffers();
    serialStream2.FlushIOBuffers();

    serialStream1 << writeString1 << std::endl;
    serialStream2 << writeString2 << std::endl;

    serialStream1.DrainWriteBuffer();
    serialStream2.DrainWriteBuffer();

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialStream1.IsDataAvailable());
    ASSERT_TRUE(serialStream2.IsDataAvailable());

    serialStream1.FlushIOBuffers();
    serialStream2.FlushIOBuffers();

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamFlushInputBuffer()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.FlushInputBuffer();
    serialStream2.FlushInputBuffer();

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    char writeByte = 'A';

    serialStream1.write(&writeByte, 1);
    serialStream2.write(&writeByte, 1);

    serialStream1.DrainWriteBuffer();
    serialStream2.DrainWriteBuffer();

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialStream1.IsDataAvailable());
    ASSERT_TRUE(serialStream2.IsDataAvailable());

    serialStream1.FlushInputBuffer();
    serialStream2.FlushInputBuffer();

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamFlushOutputBuffer()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.FlushInputBuffer();
    serialStream2.FlushInputBuffer();

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    char writeByte = 'A';

    serialStream1.write(&writeByte, 1);
    serialStream1.FlushOutputBuffer();

    serialStream2.write(&writeByte, 1);
    serialStream2.FlushOutputBuffer();

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamFlushIOBuffers()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    char writeByte = 'a';

    serialStream1.write(&writeByte, 1);
    serialStream1.FlushIOBuffers();

    ASSERT_FALSE(serialStream2.IsDataAvailable());

    serialStream2.write(&writeByte, 1);
    serialStream2.FlushIOBuffers();

    ASSERT_FALSE(serialStream1.IsDataAvailable());

    serialStream1.write(&writeByte, 1);
    serialStream2.write(&writeByte, 1);

    serialStream1.DrainWriteBuffer();
    serialStream2.DrainWriteBuffer();
    
    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialStream1.IsDataAvailable());
    ASSERT_TRUE(serialStream2.IsDataAvailable());

    serialStream1.FlushIOBuffers();
    serialStream2.FlushIOBuffers();

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    serialStream1.Close();
    serialStream2.Close();
    
    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamIsDataAvailableTest()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    char writeByte = 'a';
    char readByte = 'b';

    serialStream1.write(&writeByte, 1);
    serialStream1.DrainWriteBuffer();

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialStream2.IsDataAvailable());

    serialStream2.read(&readByte, 1);
    ASSERT_EQ(readByte, writeByte);

    serialStream2.write(&writeByte, 1);
    serialStream2.DrainWriteBuffer();

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialStream1.IsDataAvailable());

    serialStream1.read(&readByte, 1);
    ASSERT_EQ(readByte, writeByte);

    ASSERT_FALSE(serialStream1.IsDataAvailable());
    ASSERT_FALSE(serialStream2.IsDataAvailable());

    serialStream1.Close();
    serialStream2.Close();
    
    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamIsOpenTest()
{
    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());

    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetBaudRate()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    for (const auto baud_rate: baudRates)
    {
        serialStream1.SetBaudRate(baud_rate);
        serialStream2.SetBaudRate(baud_rate);

        const auto baudRate1 = serialStream1.GetBaudRate();
        const auto baudRate2 = serialStream2.GetBaudRate();

        ASSERT_EQ(baudRate1, baud_rate);
        ASSERT_EQ(baudRate2, baud_rate);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetCharacterSize()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    for (const auto char_size: characterSizes)
    {
        // @NOTE - Smaller Character Size values do not work in Linux.
        if (char_size < CharacterSize::CHAR_SIZE_7)
        {
            continue;
        }
        
        serialStream1.SetCharacterSize(char_size);
        serialStream2.SetCharacterSize(char_size);

        const auto characterSize1 = serialStream1.GetCharacterSize();
        const auto characterSize2 = serialStream2.GetCharacterSize();

        ASSERT_EQ(characterSize1, char_size);
        ASSERT_EQ(characterSize2, char_size);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetFlowControl()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    for (const auto flow_control: flowControlTypes)
    {
        serialStream1.SetFlowControl(flow_control);
        serialStream1.SetFlowControl(flow_control);

        const auto flowControl1 = serialStream1.GetFlowControl();
        const auto flowControl2 = serialStream1.GetFlowControl();

        ASSERT_EQ(flowControl1, flow_control);
        ASSERT_EQ(flowControl2, flow_control);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetParity()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    for (const auto parity: parityTypes)
    {
        serialStream1.SetParity(parity);
        serialStream2.SetParity(parity);

        const auto parity1 = serialStream1.GetParity();
        const auto parity2 = serialStream2.GetParity();

        ASSERT_EQ(parity1, parity);
        ASSERT_EQ(parity2, parity);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetStopBits()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    for (const auto stop_bits: stopBits)
    {
        serialStream1.SetStopBits(stop_bits);
        serialStream2.SetStopBits(stop_bits);

        const auto numberOfStopBits1 = serialStream1.GetStopBits();
        const auto numberOfStopBits2 = serialStream2.GetStopBits();

        ASSERT_EQ(numberOfStopBits1, stop_bits);
        ASSERT_EQ(numberOfStopBits2, stop_bits);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetVMin()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    short vMin1 = 0;
    short vMin2 = 0;

    for (short i = 0; i < 5; i++)
    {
        serialStream1.SetVMin(i);
        serialStream2.SetVMin(i);

        vMin1 = serialStream1.GetVMin();
        vMin2 = serialStream2.GetVMin();

        ASSERT_EQ(vMin1, i);
        ASSERT_EQ(vMin2, i);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetVTime()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    short vTime1 = 0;
    short vTime2 = 0;

    for (short i = 0; i < 5; i++)
    {
        serialStream1.SetVTime(i);
        serialStream2.SetVTime(i);

        vTime1 = serialStream1.GetVTime();
        vTime2 = serialStream2.GetVTime();

        ASSERT_EQ(vTime1, i);
        ASSERT_EQ(vTime2, i);
    }

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetDTR()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool dtrLine1 = false;
    bool dtrLine2 = false;

    dtrLine1 = serialStream1.GetDTR();
    dtrLine2 = serialStream2.GetDTR();

    ASSERT_TRUE(dtrLine1);
    ASSERT_TRUE(dtrLine2);

    serialStream1.SetDTR(true);
    serialStream2.SetDTR(true);

    dtrLine1 = serialStream1.GetDTR();
    dtrLine2 = serialStream2.GetDTR();

    ASSERT_TRUE(dtrLine1);
    ASSERT_TRUE(dtrLine2);

    serialStream1.SetDTR(false);
    serialStream2.SetDTR(false);

    dtrLine1 = serialStream1.GetDTR();
    dtrLine2 = serialStream2.GetDTR();

    ASSERT_FALSE(dtrLine1);
    ASSERT_FALSE(dtrLine2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetGetRTS()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool rtsLine1 = false;
    bool rtsLine2 = false;

    rtsLine1 = serialStream1.GetRTS();
    rtsLine2 = serialStream2.GetRTS();
    
    ASSERT_TRUE(rtsLine1);
    ASSERT_TRUE(rtsLine2);

    serialStream1.SetRTS(false);
    serialStream2.SetRTS(false);
    
    rtsLine1 = serialStream1.GetRTS();
    rtsLine2 = serialStream2.GetRTS();
    
    ASSERT_FALSE(rtsLine1);
    ASSERT_FALSE(rtsLine2);

    serialStream1.SetRTS(true);
    serialStream2.SetRTS(true);

    rtsLine1 = serialStream1.GetRTS();
    rtsLine2 = serialStream2.GetRTS();

    ASSERT_TRUE(rtsLine1);
    ASSERT_TRUE(rtsLine2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetRTSGetCTS()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool ctsLine1 = false;
    bool ctsLine2 = false;

    ctsLine1 = serialStream1.GetCTS();
    ctsLine2 = serialStream2.GetCTS();

    ASSERT_TRUE(ctsLine1);
    ASSERT_TRUE(ctsLine2);

    serialStream1.SetRTS(false);
    serialStream2.SetRTS(false);

    ctsLine1 = serialStream1.GetCTS();
    ctsLine2 = serialStream2.GetCTS();

    ASSERT_FALSE(ctsLine1);
    ASSERT_FALSE(ctsLine2);

    serialStream1.SetRTS(true);
    serialStream2.SetRTS(true);

    ctsLine1 = serialStream1.GetCTS();
    ctsLine2 = serialStream2.GetCTS();

    ASSERT_TRUE(ctsLine1);
    ASSERT_TRUE(ctsLine2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamSetDTRGetDSR()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool dsrStatus1 = false;
    bool dsrStatus2 = false;

    dsrStatus1 = serialStream1.GetDSR();
    dsrStatus2 = serialStream2.GetDSR();

    ASSERT_TRUE(dsrStatus1);
    ASSERT_TRUE(dsrStatus2);

    serialStream1.SetDTR(false);
    serialStream2.SetDTR(false);

    dsrStatus1 = serialStream1.GetDSR();
    dsrStatus2 = serialStream2.GetDSR();

    ASSERT_FALSE(dsrStatus1);
    ASSERT_FALSE(dsrStatus2);

    serialStream1.SetDTR(true);
    serialStream2.SetDTR(true);

    dsrStatus1 = serialStream1.GetDSR();
    dsrStatus2 = serialStream2.GetDSR();

    ASSERT_TRUE(dsrStatus1);
    ASSERT_TRUE(dsrStatus2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamGetFileDescriptor()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    int fileDescriptor1 = serialStream1.GetFileDescriptor();
    int fileDescriptor2 = serialStream2.GetFileDescriptor();

    ASSERT_GT(fileDescriptor1, 0);
    ASSERT_GT(fileDescriptor2, 0);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamGetNumberOfBytesAvailable()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    int bytesAvailable1 = 0;
    int bytesAvailable2 = 0;

    serialStream1.write(&writeByte1, 1);
    serialStream2.write(&writeByte2, 1);

    serialStream1.DrainWriteBuffer();
    serialStream2.DrainWriteBuffer();

    usleep(readBufferDelay);

    bytesAvailable1 = serialStream1.GetNumberOfBytesAvailable();
    bytesAvailable2 = serialStream2.GetNumberOfBytesAvailable();

    ASSERT_EQ(bytesAvailable1, 1);
    ASSERT_EQ(bytesAvailable2, 1);

    serialStream1.read(&readByte2, 1);
    serialStream2.read(&readByte1, 1);

    bytesAvailable1 = serialStream1.GetNumberOfBytesAvailable();
    bytesAvailable2 = serialStream2.GetNumberOfBytesAvailable();

    ASSERT_EQ(bytesAvailable1, 0);
    ASSERT_EQ(bytesAvailable2, 0);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamGetAvailableSerialPorts()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    std::vector<std::string> serialPorts1;
    std::vector<std::string> serialPorts2;

    serialPorts1.clear();
    serialPorts2.clear();

    serialPorts1 = serialStream1.GetAvailableSerialPorts();
    serialPorts2 = serialStream2.GetAvailableSerialPorts();

    int portCount1 = (int)serialPorts1.size();
    int portCount2 = (int)serialPorts2.size();

    ASSERT_GE(portCount1, 2);
    ASSERT_GE(portCount2, 2);
    ASSERT_EQ(portCount1, portCount2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamReadByteWriteByte()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    serialStream1.write(&writeByte1, 1);
    serialStream2.write(&writeByte2, 1);

    serialStream1.read(&readByte2, 1);
    serialStream2.read(&readByte1, 1);

    ASSERT_EQ(readByte1, writeByte1);
    ASSERT_EQ(readByte2, writeByte2);

    serialStream1 << writeByte1;
    serialStream2 << writeByte2;

    serialStream1.read(&readByte2, 1);
    serialStream2.read(&readByte1, 1);

    ASSERT_EQ(readByte1, writeByte1);
    ASSERT_EQ(readByte2, writeByte2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamGetLineWriteString()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    serialStream1 << writeString1 << std::endl;
    serialStream2 << writeString2 << std::endl;

    getline(serialStream1, readString2);
    getline(serialStream2, readString1);

    ASSERT_EQ(readString1, writeString1);
    ASSERT_EQ(readString2, writeString2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}

void
LibSerialTest::testSerialStreamGetWriteByte()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialStream1.IsOpen());
    ASSERT_TRUE(serialStream2.IsOpen());

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    serialStream1.write(&writeByte1, 1);
    serialStream2.write(&writeByte2, 1);

    serialStream1.get(readByte2);
    serialStream2.get(readByte1);

    ASSERT_EQ(readByte1, writeByte1);
    ASSERT_EQ(readByte2, writeByte2);

    serialStream1 << writeByte1;
    serialStream2 << writeByte2;

    serialStream1.get(readByte2);
    serialStream2.get(readByte1);

    ASSERT_EQ(readByte1, writeByte1);
    ASSERT_EQ(readByte2, writeByte2);

    serialStream1.Close();
    serialStream2.Close();

    ASSERT_FALSE(serialStream1.IsOpen());
    ASSERT_FALSE(serialStream2.IsOpen());
}


//----------------------- Serial Port Unit Tests ------------------------//

void
LibSerialTest::testSerialPortConstructors()
{
    SerialPort serialPort3(TEST_SERIAL_PORT_1);
    SerialPort serialPort4(TEST_SERIAL_PORT_2,
                           BaudRate::BAUD_9600,
                           CharacterSize::CHAR_SIZE_7,
                           FlowControl::FLOW_CONTROL_HARDWARE,
                           Parity::PARITY_EVEN,
                           StopBits::STOP_BITS_2);

    ASSERT_TRUE(serialPort3.IsOpen());
    ASSERT_TRUE(serialPort4.IsOpen());
    
    ASSERT_EQ(serialPort4.GetBaudRate(),      BaudRate::BAUD_9600);
    ASSERT_EQ(serialPort4.GetCharacterSize(), CharacterSize::CHAR_SIZE_7);
    ASSERT_EQ(serialPort4.GetFlowControl(),   FlowControl::FLOW_CONTROL_HARDWARE);
    ASSERT_EQ(serialPort4.GetParity(),        Parity::PARITY_EVEN);
    ASSERT_EQ(serialPort4.GetStopBits(),      StopBits::STOP_BITS_2);
    
    serialPort3.Close();
    serialPort4.Close();

    ASSERT_FALSE(serialPort3.IsOpen());
    ASSERT_FALSE(serialPort4.IsOpen());
}

void
LibSerialTest::testSerialPortOpenClose()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    SerialPort serialPort3;
    SerialPort serialPort4;

    ASSERT_FALSE(serialPort3.IsOpen());
    ASSERT_FALSE(serialPort4.IsOpen());

    bool exclusiveUseTestPass = false;

    try
    {
        serialPort3.Open(TEST_SERIAL_PORT_1);
        serialPort4.Open(TEST_SERIAL_PORT_2);
    }
    catch (...)
    {
        exclusiveUseTestPass = true;
    }

    ASSERT_TRUE(exclusiveUseTestPass);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortDrainWriteBuffer()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.FlushIOBuffers();
    serialPort2.FlushIOBuffers();

    serialPort1.Write(writeString1);
    serialPort2.Write(writeString2);

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialPort1.IsDataAvailable());
    ASSERT_TRUE(serialPort2.IsDataAvailable());

    serialPort1.FlushIOBuffers();
    serialPort2.FlushIOBuffers();

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortFlushInputBuffer()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.FlushInputBuffer();
    serialPort2.FlushInputBuffer();

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    char writeByte = 'A';

    serialPort1.WriteByte(writeByte);
    serialPort2.WriteByte(writeByte);

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialPort1.IsDataAvailable());
    ASSERT_TRUE(serialPort2.IsDataAvailable());

    serialPort1.FlushInputBuffer();
    serialPort2.FlushInputBuffer();

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortFlushOutputBuffer()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.FlushInputBuffer();
    serialPort2.FlushInputBuffer();

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    char writeByte = 'A';
    
    serialPort1.WriteByte(writeByte);
    serialPort1.FlushOutputBuffer();

    serialPort2.WriteByte(writeByte);
    serialPort2.FlushOutputBuffer();

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    serialPort1.Close();
    serialPort2.Close();
    
    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortFlushIOBuffers()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    char writeByte = 'a';

    serialPort1.WriteByte(writeByte);
    serialPort1.FlushIOBuffers();

    ASSERT_FALSE(serialPort2.IsDataAvailable());

    serialPort2.WriteByte(writeByte);
    serialPort2.FlushIOBuffers();

    ASSERT_FALSE(serialPort1.IsDataAvailable());

    serialPort1.WriteByte(writeByte);
    serialPort2.WriteByte(writeByte);

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();
    
    // Allow time for the read buffers to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialPort1.IsDataAvailable());
    ASSERT_TRUE(serialPort2.IsDataAvailable());

    serialPort1.FlushIOBuffers();
    serialPort2.FlushIOBuffers();

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortIsDataAvailableTest()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    char writeByte = 'a';
    char readByte = 'b';

    serialPort1.WriteByte(writeByte);
    serialPort1.DrainWriteBuffer();

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialPort2.IsDataAvailable());

    serialPort2.ReadByte(readByte, 1);
    ASSERT_EQ(readByte, writeByte);

    serialPort2.WriteByte(writeByte);
    serialPort2.DrainWriteBuffer();

    // Allow time for the read buffer to show that data is available.
    usleep(readBufferDelay);

    ASSERT_TRUE(serialPort1.IsDataAvailable());

    serialPort1.ReadByte(readByte, 1);
    ASSERT_EQ(readByte, writeByte);

    ASSERT_FALSE(serialPort1.IsDataAvailable());
    ASSERT_FALSE(serialPort2.IsDataAvailable());

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortIsOpenTest()
{
    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());

    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetBaudRate()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    for(const auto baud_rate: baudRates)
    {
        serialPort1.SetBaudRate(baud_rate);
        serialPort2.SetBaudRate(baud_rate);

        const auto baudRate1 = serialPort1.GetBaudRate();
        const auto baudRate2 = serialPort1.GetBaudRate();

        ASSERT_EQ(baudRate1, baud_rate);
        ASSERT_EQ(baudRate2, baud_rate);
    }

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetCharacterSize()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    for(const auto char_size: characterSizes)
    {
        // @NOTE - Smaller CharSize values appear to be invalid on x86 Linux.
        if (char_size < CharacterSize::CHAR_SIZE_7)
        {
            continue;
        }

        serialPort1.SetCharacterSize(char_size);
        serialPort2.SetCharacterSize(char_size);

        const auto characterSize1 = serialPort1.GetCharacterSize();
        const auto characterSize2 = serialPort2.GetCharacterSize();

        ASSERT_EQ(characterSize1, char_size);
        ASSERT_EQ(characterSize2, char_size);
    }

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetFlowControl()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    for (const auto flow_control: flowControlTypes)
    {
        // @NOTE - FLOW_CONTROL_SOFT flow control appears to be invalid on x86 Linux.
        if (flow_control == FlowControl::FLOW_CONTROL_SOFTWARE)
        {
            continue;
        }

        serialPort1.SetFlowControl(flow_control);
        serialPort2.SetFlowControl(flow_control);

        const auto flowControl1 = serialPort1.GetFlowControl();
        const auto flowControl2 = serialPort2.GetFlowControl();

        ASSERT_EQ(flowControl1, flow_control);
        ASSERT_EQ(flowControl2, flow_control);
    }

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetParity()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    for (const auto parity: parityTypes)
    {
        serialPort1.SetParity(parity);
        serialPort2.SetParity(parity);

        const auto parity1 = serialPort1.GetParity();
        const auto parity2 = serialPort2.GetParity();

        ASSERT_EQ(parity1, parity);
        ASSERT_EQ(parity2, parity);
    }

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetStopBits()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    for (const auto stop_bits: stopBits)
    {
        serialPort1.SetStopBits(stop_bits);
        serialPort2.SetStopBits(stop_bits);

        const auto stopBits1 = serialPort1.GetStopBits();
        const auto stopBits2 = serialPort2.GetStopBits();

        ASSERT_EQ(stopBits1, stop_bits);
        ASSERT_EQ(stopBits2, stop_bits);
    }

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetVMin()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    ASSERT_TRUE(serialPort1.IsOpen());

    for (short i = 0; i < 5; i++)
    {
        serialPort1.SetVMin(i);
        short vMin = serialPort1.GetVMin();
        ASSERT_EQ(vMin, i);
    }

    serialPort1.Close();
    ASSERT_FALSE(serialPort1.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetVTime()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    ASSERT_TRUE(serialPort1.IsOpen());

    for (short i = 0; i < 5; i++)
    {
        serialPort1.SetVTime(i);
        short vTime = serialPort1.GetVTime();
        ASSERT_EQ(vTime, i);
    }

    serialPort1.Close();
    ASSERT_FALSE(serialPort1.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetDTR()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool dtrLine1 = false;
    bool dtrLine2 = false;

    dtrLine1 = serialPort1.GetDTR();
    dtrLine2 = serialPort2.GetDTR();

    ASSERT_TRUE(dtrLine1);
    ASSERT_TRUE(dtrLine2);

    serialPort1.SetDTR(false);
    serialPort2.SetDTR(false);

    dtrLine1 = serialPort1.GetDTR();
    dtrLine2 = serialPort2.GetDTR();

    ASSERT_FALSE(dtrLine1);
    ASSERT_FALSE(dtrLine2);

    serialPort1.SetDTR(true);
    serialPort2.SetDTR(true);

    dtrLine1 = serialPort1.GetDTR();
    dtrLine2 = serialPort2.GetDTR();

    ASSERT_TRUE(dtrLine1);
    ASSERT_TRUE(dtrLine2);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetGetRTS()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool rtsLine1 = false;
    bool rtsLine2 = false;

    rtsLine1 = serialPort1.GetRTS();
    rtsLine2 = serialPort2.GetRTS();
    
    ASSERT_TRUE(rtsLine1);
    ASSERT_TRUE(rtsLine2);

    serialPort1.SetRTS(false);
    serialPort2.SetRTS(false);
    
    rtsLine1 = serialPort1.GetRTS();
    rtsLine2 = serialPort2.GetRTS();
    
    ASSERT_FALSE(rtsLine1);
    ASSERT_FALSE(rtsLine2);

    serialPort1.SetRTS(true);
    serialPort2.SetRTS(true);

    rtsLine1 = serialPort1.GetRTS();
    rtsLine2 = serialPort2.GetRTS();

    ASSERT_TRUE(rtsLine1);
    ASSERT_TRUE(rtsLine2);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetRTSGetCTS()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool ctsLine1 = false;
    bool ctsLine2 = false;

    ctsLine1 = serialPort1.GetCTS();
    ctsLine2 = serialPort2.GetCTS();

    ASSERT_TRUE(ctsLine1);
    ASSERT_TRUE(ctsLine2);

    serialPort1.SetRTS(false);
    serialPort2.SetRTS(false);

    ctsLine1 = serialPort1.GetCTS();
    ctsLine2 = serialPort2.GetCTS();

    ASSERT_FALSE(ctsLine1);
    ASSERT_FALSE(ctsLine2);

    serialPort1.SetRTS(true);
    serialPort2.SetRTS(true);

    ctsLine1 = serialPort1.GetCTS();
    ctsLine2 = serialPort2.GetCTS();

    ASSERT_TRUE(ctsLine1);
    ASSERT_TRUE(ctsLine2);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortSetDTRGetDSR()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    bool dsrStatus1 = false;
    bool dsrStatus2 = false;

    dsrStatus1 = serialPort1.GetDSR();
    dsrStatus2 = serialPort1.GetDSR();

    ASSERT_TRUE(dsrStatus1);
    ASSERT_TRUE(dsrStatus2);

    serialPort1.SetDTR(false);
    serialPort2.SetDTR(false);

    dsrStatus1 = serialPort1.GetDSR();
    dsrStatus2 = serialPort2.GetDSR();

    ASSERT_FALSE(dsrStatus1);
    ASSERT_FALSE(dsrStatus2);

    serialPort1.SetDTR(true);
    serialPort2.SetDTR(true);

    dsrStatus1 = serialPort1.GetDSR();
    dsrStatus2 = serialPort2.GetDSR();

    ASSERT_TRUE(dsrStatus1);
    ASSERT_TRUE(dsrStatus2);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortGetFileDescriptor()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    int fileDescriptor1 = serialPort1.GetFileDescriptor();
    int fileDescriptor2 = serialPort2.GetFileDescriptor();

    ASSERT_GT(fileDescriptor1, 0);
    ASSERT_GT(fileDescriptor2, 0);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortGetNumberOfBytesAvailable()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    char writeByte1 = 'a';
    char writeByte2 = 'A';

    char readByte1  = 'b';
    char readByte2  = 'B';

    int bytesAvailable1 = 0;
    int bytesAvailable2 = 0;

    bytesAvailable1 = serialPort1.GetNumberOfBytesAvailable();
    bytesAvailable2 = serialPort2.GetNumberOfBytesAvailable();

    serialPort1.WriteByte(writeByte1);
    serialPort2.WriteByte(writeByte2);

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    usleep(readBufferDelay);

    bytesAvailable1 = serialPort1.GetNumberOfBytesAvailable();
    bytesAvailable2 = serialPort2.GetNumberOfBytesAvailable();

    ASSERT_EQ(bytesAvailable1, 1);
    ASSERT_EQ(bytesAvailable2, 1);

    serialPort1.ReadByte(readByte2, 1);
    serialPort2.ReadByte(readByte1, 1);

    bytesAvailable1 = serialPort1.GetNumberOfBytesAvailable();
    bytesAvailable2 = serialPort2.GetNumberOfBytesAvailable();

    ASSERT_EQ(bytesAvailable1, 0);
    ASSERT_EQ(bytesAvailable2, 0);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortGetAvailableSerialPorts()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    std::vector<std::string> serialPorts1;
    std::vector<std::string> serialPorts2;

    serialPorts1.clear();
    serialPorts2.clear();

    serialPorts1 = serialPort1.GetAvailableSerialPorts();
    serialPorts2 = serialPort2.GetAvailableSerialPorts();

    int portCount1 = (int)serialPorts1.size();
    int portCount2 = (int)serialPorts2.size();

    ASSERT_GE(portCount1, 2);
    ASSERT_GE(portCount2, 2);
    ASSERT_EQ(portCount1, portCount2);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortReadDataBufferWriteDataBuffer()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    const size_t data_count = 75;

    bool timeOutTestPass = false;

    DataBuffer writeVector1;
    DataBuffer writeVector2;

    DataBuffer readVector1;
    DataBuffer readVector2;

    // Test using ASCII characters.
    for (unsigned char i = 0; i < data_count; i++)
    {
        writeVector1.push_back((char)(48 + i));
        writeVector2.push_back((char)(122 - i));
    }

    serialPort1.Write(writeVector1);
    serialPort2.Write(writeVector2);

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    serialPort1.Read(readVector2, data_count, timeOutMilliseconds);
    serialPort2.Read(readVector1, data_count, timeOutMilliseconds);

    ASSERT_EQ(readVector1, writeVector1);
    ASSERT_EQ(readVector2, writeVector2);

    readVector1.clear();
    readVector2.clear();

    try
    {
        serialPort1.Write(writeVector1);

        serialPort1.DrainWriteBuffer();

        serialPort2.Read(readVector1, 0, 75);
    }
    catch(...)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass);
    ASSERT_EQ(readVector1, writeVector1);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortReadStringWriteString()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    bool timeOutTestPass = false;

    serialPort1.Write(writeString1);
    serialPort2.Write(writeString2);

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    serialPort1.Read(readString2, writeString2.size(), timeOutMilliseconds);
    serialPort2.Read(readString1, writeString1.size(), timeOutMilliseconds);

    ASSERT_EQ(readString1, writeString1);
    ASSERT_EQ(readString2, writeString2);

    timeOutTestPass = false;

    readString1.clear();
    readString2.clear();

    try
    {
        serialPort1.Write(writeString1);

        serialPort1.DrainWriteBuffer();

        serialPort2.Read(readString1, 0, 75);
    }
    catch(...)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass);
    ASSERT_EQ(readString1, writeString1);
    
    serialPort1.Close();
    serialPort2.Close();
    
    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortReadByteWriteByte()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    bool timeOutTestPass = false;

    char writeByte1 = 'a';
    unsigned char writeByte2 = 'A';

    char readByte1  = 'b';
    unsigned char readByte2  = 'B';

    serialPort1.WriteByte(writeByte1);
    serialPort2.WriteByte(writeByte2);

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    serialPort1.ReadByte(readByte2, timeOutMilliseconds);
    serialPort2.ReadByte(readByte1, timeOutMilliseconds);

    ASSERT_EQ(readByte1, writeByte1);
    ASSERT_EQ(readByte2, writeByte2);

    try
    {
        serialPort1.ReadByte(readByte1, 1);
        serialPort2.ReadByte(readByte2, 1);
    }
    catch(ReadTimeout)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialPortReadLineWriteString()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort2.Open(TEST_SERIAL_PORT_2);

    ASSERT_TRUE(serialPort1.IsOpen());
    ASSERT_TRUE(serialPort2.IsOpen());

    bool timeOutTestPass = false;

    serialPort1.Write(writeString1 + '\n');
    serialPort2.Write(writeString2 + '\n');

    serialPort1.DrainWriteBuffer();
    serialPort2.DrainWriteBuffer();

    serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);
    serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);

    ASSERT_EQ(readString1, writeString1 + '\n');
    ASSERT_EQ(readString2, writeString2 + '\n');

    try
    {
        serialPort1.ReadLine(readString2, '\n', 1);
        serialPort2.ReadLine(readString1, '\n', 1);
    }
    catch(ReadTimeout)
    {
        timeOutTestPass = true;
    }

    ASSERT_TRUE(timeOutTestPass);

    serialPort1.Close();
    serialPort2.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialPort2.IsOpen());
}

void
LibSerialTest::testSerialStreamToSerialPortReadWrite()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    ASSERT_TRUE(serialPort1.IsOpen());

    serialStream1.Open(TEST_SERIAL_PORT_2);
    ASSERT_TRUE(serialStream1.IsOpen());

    const auto baud_rate = BaudRate::BAUD_115200 ;
    serialPort1.SetBaudRate(baud_rate);
    serialStream1.SetBaudRate(baud_rate);

    BaudRate baudRate1 = serialPort1.GetBaudRate();
    BaudRate baudRate2 = serialStream1.GetBaudRate();

    ASSERT_EQ(baudRate1, baud_rate);
    ASSERT_EQ(baudRate2, baud_rate);

    serialStream1 << writeString1 << std::endl;
    serialPort1.ReadLine(readString1, '\n', timeOutMilliseconds);

    ASSERT_EQ(readString1, writeString1 + '\n');
    ASSERT_EQ(readString1.size(), writeString1.size() + 1);

    serialPort1.Write(writeString2 + '\n');
    serialPort1.DrainWriteBuffer();
    getline(serialStream1, readString2);

    ASSERT_EQ(readString2, writeString2);

    serialPort1.Close();
    serialStream1.Close();

    ASSERT_FALSE(serialPort1.IsOpen());
    ASSERT_FALSE(serialStream1.IsOpen());
}

void
LibSerialTest::serialStream1ThreadLoop()
{
    serialStream1.Open(TEST_SERIAL_PORT_1);
    serialStream1.SetBaudRate(BaudRate::BAUD_115200);
    serialStream1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds();
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        serialStream1 << writeString1 << std::endl;
        serialStream1.DrainWriteBuffer();

        if (serialStream1.IsDataAvailable())
        {
            alarm(5);   // Set a system alarm in case getline() blocks longer than 5 seconds.
            getline(serialStream1, readString2);
            alarm(0);   // Deactivate the alarm.
            
            if(readString2 != writeString2)
            {
                mutex.lock();
                failureRate++;
                mutex.unlock();
            }
        }

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;

        mutex.lock();
        loopCount++;
        mutex.unlock();
    }

    serialStream1.Close();
    return;
}

void
LibSerialTest::serialStream2ThreadLoop()
{
    serialStream2.Open(TEST_SERIAL_PORT_2);
    serialStream2.SetBaudRate(BaudRate::BAUD_115200);
    serialStream2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds();
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        serialStream2 << writeString2 << std::endl;
        serialStream2.DrainWriteBuffer();

        if (serialStream2.IsDataAvailable())
        {
            alarm(5);   // Set a system alarm in case getline() blocks longer than 5 seconds.
            getline(serialStream2, readString1);
            alarm(0);   // Deactivate the alarm.

            if(readString1 != writeString1)
            {
                mutex.lock();
                failureRate++;
                mutex.unlock();
            }
        }
        
        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;
        
        mutex.lock();
        loopCount++;
        mutex.unlock();
    }

    serialStream2.Close();
    return;
}

void
LibSerialTest::serialPort1ThreadLoop()
{
    serialPort1.Open(TEST_SERIAL_PORT_1);
    serialPort1.SetBaudRate(BaudRate::BAUD_115200);
    serialPort1.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

    size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds();
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
       
        serialPort1.Write(writeString1 + '\n');
        serialPort1.DrainWriteBuffer();

        if (serialPort1.IsDataAvailable())
        {
            try
            {
                serialPort1.ReadLine(readString2, '\n', timeOutMilliseconds);

                if(readString2 != writeString2 + '\n')
                {
                    mutex.lock();
                    failureRate++;
                    mutex.unlock();
                }
            }
            catch(...)
            {
            }
        }

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;

        mutex.lock();
        loopCount++;
        mutex.unlock();
    }

    serialPort1.Close();
    return;
}

void
LibSerialTest::serialPort2ThreadLoop()
{
    serialPort2.Open(TEST_SERIAL_PORT_2);
    serialPort2.SetBaudRate(BaudRate::BAUD_115200);
    serialPort2.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);

    tcflush(serialPort1.GetFileDescriptor(), TCIOFLUSH);

    size_t threadLoopStartTimeMilliseconds = getTimeInMilliSeconds();
    size_t timeElapsedMilliSeconds = 0;

    while (timeElapsedMilliSeconds < timeOutMilliseconds)
    {
        serialPort2.Write(writeString2 + '\n');
        serialPort2.DrainWriteBuffer();

        if (serialPort2.IsDataAvailable())
        {
            try
            {
                serialPort2.ReadLine(readString1, '\n', timeOutMilliseconds);

                if(readString1 != writeString1 + '\n')
                {
                    mutex.lock();
                    failureRate++;
                    mutex.unlock();
                }
            }
            catch(...)
            {
            }
        }

        timeElapsedMilliSeconds = getTimeInMilliSeconds() - threadLoopStartTimeMilliseconds;

        mutex.lock();
        loopCount++;
        mutex.unlock();
    }

    serialPort2.Close();
    return;
}

void
LibSerialTest::testMultiThreadSerialStreamReadWrite()
{
    failureRate = 0;
    loopCount = 0;

    std::thread serialStream1Thread(&LibSerialTest::serialStream1ThreadLoop, this);
    std::thread serialStream2Thread(&LibSerialTest::serialStream2ThreadLoop, this);

    serialStream1Thread.join();
    serialStream2Thread.join();
}

void
LibSerialTest::testMultiThreadSerialPortReadWrite()
{
    failureRate = 0;
    loopCount = 0;
    
    std::thread serialPort1Thread(&LibSerialTest::serialPort1ThreadLoop, this);
    std::thread serialPort2Thread(&LibSerialTest::serialPort2ThreadLoop, this);

    serialPort1Thread.join();
    serialPort2Thread.join();
}


//------------------------ Serial Stream Unit Tests -------------------------//

TEST_F(LibSerialTest, testSerialStreamConstructors)
{
    SCOPED_TRACE("Serial Stream Constructor Tests");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamConstructors();
    }  
}

TEST_F(LibSerialTest, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open() and Close() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialStreamDrainWriteBuffer)
{
    SCOPED_TRACE("Serial Stream DrainWriteBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamDrainWriteBuffer();
    }
}

TEST_F(LibSerialTest, testSerialStreamFlushInputBuffer)
{
    SCOPED_TRACE("Serial Stream FlushInputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamFlushInputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialStreamFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Stream FlushOutputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamFlushOutputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialStreamFlushIOBuffers)
{
    SCOPED_TRACE("Serial Stream FlushIOBuffers() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamFlushIOBuffers();
    }
}

TEST_F(LibSerialTest, testSerialStreamIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Stream IsDataAvailable() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialStreamIsOpenTest)
{
    SCOPED_TRACE("Serial Stream IsOpen() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamIsOpenTest();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Stream SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Stream SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetDTR)
{
    SCOPED_TRACE("Serial Stream SetDTR() and GetDTR() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetRTS)
{
    SCOPED_TRACE("Serial Stream SetRTS() and GetRTS() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetRTSGetCTS)
{
    SCOPED_TRACE("Serial Stream SetRTS() and GetCTS() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetRTSGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetDTRGetDSR)
{
    SCOPED_TRACE("Serial Stream SetDTR() and GetDSR() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetDTRGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetFileDescriptor)
{
    SCOPED_TRACE("Serial Stream GetFileDescriptor() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetFileDescriptor();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetNumberOfBytesAvailable)
{
    SCOPED_TRACE("Serial Stream GetNumberOfBytesAvailable() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetNumberOfBytesAvailable();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetAvailableSerialPorts)
{
    SCOPED_TRACE("Serial Stream GetAvailableSerialPorts() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetAvailableSerialPorts();
    }
}

TEST_F(LibSerialTest, testSerialStreamReadByteWriteByte)
{
    SCOPED_TRACE("Serial Stream Read() and WriteByte() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamReadByteWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetLineWriteString)
{
    SCOPED_TRACE("Serial Stream GetLine() and Write(string) Test");
        
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetLineWriteString();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetWriteByte)
{
    SCOPED_TRACE("Serial Stream Get() and WriteByte() Test");
        
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamGetWriteByte();
    }
}

//-------------------------- Serial Port Unit Tests -------------------------//

TEST_F(LibSerialTest, testSerialPortConstructors)
{
    SCOPED_TRACE("Serial Port Constructors Tests");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortConstructors();
    }  
}

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open() and Close() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialPortDrainWriteBuffer)
{
    SCOPED_TRACE("Serial Port DrainWriteBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortDrainWriteBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushInputBuffer)
{
    SCOPED_TRACE("Serial Port FlushInputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushInputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushOutputBuffer)
{
    SCOPED_TRACE("Serial Port FlushOutputBuffer() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushOutputBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortFlushIOBuffers)
{
    SCOPED_TRACE("Serial Port FlushIOBuffers() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortFlushIOBuffers();
    }
}

TEST_F(LibSerialTest, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port IsDataAvailable() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialPortIsOpenTest)
{
    SCOPED_TRACE("Serial Port IsOpen() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortIsOpenTest();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDTR() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetRTS() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialPortSetRTSGetCTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetCTS() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetRTSGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialPortSetDTRGetDSR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDSR() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortSetDTRGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialPortGetFileDescriptor)
{
    SCOPED_TRACE("Serial Port GetFileDescriptor() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortGetFileDescriptor();
    }
}

TEST_F(LibSerialTest, testSerialPortGetNumberOfBytesAvailable)
{
    SCOPED_TRACE("Serial Port GetNumberOfBytesAvailable() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortGetNumberOfBytesAvailable();
    }
}

TEST_F(LibSerialTest, testSerialPortGetAvailableSerialPorts)
{
    SCOPED_TRACE("Serial Port GetAvailableSerialPorts() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortGetAvailableSerialPorts();
    }
}

TEST_F(LibSerialTest, testSerialPortReadDataBufferWriteDataBuffer)
{
    SCOPED_TRACE("Serial Port Read(DataBuffer) and Write(DataBuffer) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadDataBufferWriteDataBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortReadStringWriteString)
{
    SCOPED_TRACE("Serial Port Read(string) and Write(string) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadStringWriteString();
    }
}

TEST_F(LibSerialTest, testSerialPortReadByteWriteByte)
{
    SCOPED_TRACE("Serial Port ReadByte() and WriteByte() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadByteWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialPortReadLineWriteString)
{
    SCOPED_TRACE("Serial Port ReadLine() and Write(string) Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadLineWriteString();
    }
}


//----------------- Serial Stream to Serial Port Unit Test ------------------//

TEST_F(LibSerialTest, testSerialStreamToSerialPortReadWrite)
{
    SCOPED_TRACE("Serial Stream To Serial Port Read and Write Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamToSerialPortReadWrite();
    }
}


//----------------------- Multiple Thread Unit Tests ------------------------//

TEST_F(LibSerialTest, testMultiThreadSerialStreamReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Stream Communication.");

    std::cout << "Note: This test calls getline() which can block indefinitely "
              << "if a newline character is not recieved." << std::endl;

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testMultiThreadSerialStreamReadWrite();
    }

    double failRate = 100. * (double)failureRate / (double)loopCount;
    
    // If the serial communication fail rate is greater than 0.001% consider it a failed test.
    if (failRate > 0.001)
    {
        std::cout << "\t     SerialStream Failure Rate = " << failRate << "%" << std::endl;
        ADD_FAILURE();
    }
}

TEST_F(LibSerialTest, testMultiThreadSerialPortReadWrite)
{
    SCOPED_TRACE("Test Multi-Thread Serial Port Communication.");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testMultiThreadSerialPortReadWrite();
    }

    double failRate = 100. * (double)failureRate / (double)loopCount;
    
    // If the serial communication fail rate is greater than 0.001% consider it a failed test.
    if (failRate > 0.001)
    {
        std::cout << "\t     SerialPort Failure Rate = " << failRate << "%" << std::endl;
        ADD_FAILURE();
    }
}
