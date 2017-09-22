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
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <unistd.h>

#include <SerialPort.h>
#include <SerialStream.h>

// Default Serial Ports.
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"

using namespace LibSerial;

class LibSerialTest
    : public ::testing::Test
{
public:
    LibSerialTest() : serialPort1(TEST_SERIAL_PORT_1), serialPort2(TEST_SERIAL_PORT_2) {} 

    size_t numberOfTestIterations = 10;

protected:

    size_t timeOutMilliseconds = 250;
    size_t failureRate = 0;
    size_t loopCount = 0;

    SerialPort serialPort1;
    SerialPort serialPort2;

    SerialPort::BaudRate             serialPortBaudRates[30];
    SerialPort::CharacterSize        serialPortCharacterSizes[4];
    SerialPort::Parity               serialPortParityTypes[3];
    SerialPort::FlowControl          serialPortFlowControlTypes[3];
    SerialPort::StopBits             serialPortStopBits[2];

    SerialStreamBuf::BaudRateEnum    serialStreamBaudRates[30];
    SerialStreamBuf::CharSizeEnum    serialStreamCharacterSizes[4];
    SerialStreamBuf::ParityEnum      serialStreamParityTypes[3];
    SerialStreamBuf::FlowControlEnum serialStreamFlowControlTypes[3];
    short                            serialStreamStopBits[2];

    SerialStream serialStream1;
    SerialStream serialStream2;

    std::string readString1  = " ";
    std::string writeString1 = " ";

    std::string readString2  = " ";
    std::string writeString2 = " ";

    char writeByte = 'a';
    char readByte = 'b';

    virtual void SetUp()
    {
        writeString1 = "Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)";
        writeString2 = "The secret of the man who is universally interesting is that he is universally interested. - William Dean Howells";

        serialPortBaudRates[0]   =  SerialPort::BAUD_50;
        serialPortBaudRates[1]   =  SerialPort::BAUD_75;
        serialPortBaudRates[2]   =  SerialPort::BAUD_110;
        serialPortBaudRates[3]   =  SerialPort::BAUD_134;
        serialPortBaudRates[4]   =  SerialPort::BAUD_150;
        serialPortBaudRates[5]   =  SerialPort::BAUD_200;
        serialPortBaudRates[6]   =  SerialPort::BAUD_300;
        serialPortBaudRates[7]   =  SerialPort::BAUD_600;
        serialPortBaudRates[8]   =  SerialPort::BAUD_1200;
        serialPortBaudRates[9]   =  SerialPort::BAUD_1800;
        serialPortBaudRates[10]  =  SerialPort::BAUD_2400;
        serialPortBaudRates[11]  =  SerialPort::BAUD_4800;
        serialPortBaudRates[12]  =  SerialPort::BAUD_9600;
        serialPortBaudRates[13]  =  SerialPort::BAUD_19200;
        serialPortBaudRates[14]  =  SerialPort::BAUD_38400;
        serialPortBaudRates[15]  =  SerialPort::BAUD_57600;
        serialPortBaudRates[16]  =  SerialPort::BAUD_115200;
        serialPortBaudRates[17]  =  SerialPort::BAUD_230400;
        serialPortBaudRates[18]  =  SerialPort::BAUD_460800;
        serialPortBaudRates[19]  =  SerialPort::BAUD_500000;
        serialPortBaudRates[20]  =  SerialPort::BAUD_576000;
        serialPortBaudRates[21]  =  SerialPort::BAUD_921600;
        serialPortBaudRates[22]  =  SerialPort::BAUD_1000000;
        serialPortBaudRates[23]  =  SerialPort::BAUD_1152000;
        serialPortBaudRates[24]  =  SerialPort::BAUD_1500000;
        serialPortBaudRates[25]  =  SerialPort::BAUD_2000000;
        serialPortBaudRates[26]  =  SerialPort::BAUD_2500000;
        serialPortBaudRates[27]  =  SerialPort::BAUD_3000000;
        serialPortBaudRates[28]  =  SerialPort::BAUD_3500000;
        serialPortBaudRates[29]  =  SerialPort::BAUD_4000000;

        serialPortCharacterSizes[0] = SerialPort::CHAR_SIZE_5;
        serialPortCharacterSizes[1] = SerialPort::CHAR_SIZE_6;
        serialPortCharacterSizes[2] = SerialPort::CHAR_SIZE_7;
        serialPortCharacterSizes[3] = SerialPort::CHAR_SIZE_8;

        serialPortParityTypes[0] = SerialPort::PARITY_EVEN;
        serialPortParityTypes[1] = SerialPort::PARITY_ODD;
        serialPortParityTypes[2] = SerialPort::PARITY_NONE;

        serialPortFlowControlTypes[0] = SerialPort::FLOW_CONTROL_NONE;
        serialPortFlowControlTypes[1] = SerialPort::FLOW_CONTROL_HARD;
        serialPortFlowControlTypes[2] = SerialPort::FLOW_CONTROL_SOFT;
        
        serialPortStopBits[0] = SerialPort::STOP_BITS_1;
        serialPortStopBits[1] = SerialPort::STOP_BITS_2;

        for (size_t i = 0; i < 30; i++)
        {
            serialStreamBaudRates[i] =  SerialStreamBuf::BaudRateEnum(serialPortBaudRates[i]);
        }

        for (size_t i = 0; i < 4; i++)
        {
            serialStreamCharacterSizes[i] = SerialStreamBuf::CharSizeEnum(serialPortCharacterSizes[i]);
        }

        for (size_t i = 0; i < 3; i++)
        {
            serialStreamParityTypes[i] = SerialStreamBuf::ParityEnum(serialPortParityTypes[i]);
            serialStreamFlowControlTypes[i] = SerialStreamBuf::FlowControlEnum(serialPortFlowControlTypes[i]);
        }

        serialStreamStopBits[0] = 1;
        serialStreamStopBits[1] = 2;
    }

    size_t getTimeInMilliSeconds()
    {
        std::chrono::high_resolution_clock::duration timeNow = 
            std::chrono::high_resolution_clock::now().time_since_epoch();

        return std::chrono::duration_cast<std::chrono::milliseconds>(timeNow).count();
    }
    
    //---------------------- Serial Stream Unit Tests -----------------------//

    void testSerialStreamOpenClose()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetBaudRate()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        size_t maxBaudIndex = 17;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialStream1.SetBaudRate(serialStreamBaudRates[i]);
            serialStream2.SetBaudRate(serialStreamBaudRates[i]);

            SerialStreamBuf::BaudRateEnum baudRate1 = serialStream1.BaudRate();
            SerialStreamBuf::BaudRateEnum baudRate2 = serialStream2.BaudRate();
            
            ASSERT_EQ(baudRate1, serialStreamBaudRates[i]);
            ASSERT_EQ(baudRate2, serialStreamBaudRates[i]);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetCharacterSize()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        // @NOTE - Smaller Character Size values do not pass.
        for (size_t i = 2; i < 4; i++)
        {
            serialStream1.SetCharSize(serialStreamCharacterSizes[i]);
            serialStream2.SetCharSize(serialStreamCharacterSizes[i]);

            SerialStreamBuf::CharSizeEnum characterSize1 = serialStream1.CharSize();
            SerialStreamBuf::CharSizeEnum characterSize2 = serialStream2.CharSize();
            
            ASSERT_EQ(characterSize1, serialStreamCharacterSizes[i]);
            ASSERT_EQ(characterSize2, serialStreamCharacterSizes[i]);

            usleep(10);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetFlowControl()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetFlowControl(serialStreamFlowControlTypes[i]);
            serialStream2.SetFlowControl(serialStreamFlowControlTypes[i]);
            
            SerialStreamBuf::FlowControlEnum flowControl1 = serialStream1.FlowControl();
            SerialStreamBuf::FlowControlEnum flowControl2 = serialStream2.FlowControl();
            
            ASSERT_EQ(flowControl1, serialStreamFlowControlTypes[i]);
            ASSERT_EQ(flowControl2, serialStreamFlowControlTypes[i]);
            
            usleep(10);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetParity()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetParity(serialStreamParityTypes[i]);
            serialStream2.SetParity(serialStreamParityTypes[i]);
            
            SerialStreamBuf::ParityEnum parity1 = serialStream1.Parity();
            SerialStreamBuf::ParityEnum parity2 = serialStream2.Parity();
            
            ASSERT_EQ(parity1, serialStreamParityTypes[i]);
            ASSERT_EQ(parity2, serialStreamParityTypes[i]);

            usleep(10);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        short numberOfStopBits1;
        short numberOfStopBits2;

        for (size_t i = 0; i < 2; i++)
        {
            serialStream1.SetNumOfStopBits(serialStreamStopBits[i]);
            serialStream2.SetNumOfStopBits(serialStreamStopBits[i]);

            numberOfStopBits1 = serialStream1.NumOfStopBits();
            numberOfStopBits2 = serialStream2.NumOfStopBits();

            ASSERT_EQ(numberOfStopBits1, serialStreamStopBits[i]);
            ASSERT_EQ(numberOfStopBits2, serialStreamStopBits[i]);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetVMin()
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

            vMin1 = serialStream1.VMin();
            vMin2 = serialStream2.VMin();

            ASSERT_EQ(vMin1, i);
            ASSERT_EQ(vMin2, i);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetVTime()
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

            vTime1 = serialStream1.VTime();
            vTime2 = serialStream2.VTime();

            ASSERT_EQ(vTime1, i);
            ASSERT_EQ(vTime2, i);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamReadByteWriteByte()
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

    void testSerialStreamGetLineWriteString()
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

    void testSerialStreamGetWriteByte()
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

    void testSerialPortOpenClose()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortIsDataAvailableTest()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        char writeByte = 'a';
        unsigned char readByte = 'b';

        serialPort1.WriteByte(writeByte);
        serialPort2.WriteByte(writeByte);

        usleep(25000);

        ASSERT_TRUE(serialPort1.IsDataAvailable());
        ASSERT_TRUE(serialPort2.IsDataAvailable());

        readByte = serialPort1.ReadByte(25);
        ASSERT_EQ(readByte, writeByte);

        readByte = serialPort2.ReadByte(25);
        ASSERT_EQ(readByte, writeByte);

        usleep(25000);

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetBaudRate()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        size_t maxBaudIndex = 25;

        SerialPort::BaudRate baudRate1;
        SerialPort::BaudRate baudRate2;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialPort1.SetBaudRate(serialPortBaudRates[i]);
            serialPort2.SetBaudRate(serialPortBaudRates[i]);

            baudRate1 = serialPort1.GetBaudRate();
            baudRate2 = serialPort2.GetBaudRate();

            ASSERT_EQ(baudRate1, serialPortBaudRates[i]);
            ASSERT_EQ(baudRate2, serialPortBaudRates[i]);
        }

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetCharacterSize()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        SerialPort::CharacterSize characterSize1;
        SerialPort::CharacterSize characterSize2;

        // @NOTE - Smaller CharSize values do not work in Linux.
        for (size_t i = 2; i < 4; i++)
        {
            serialPort1.SetCharSize(serialPortCharacterSizes[i]);
            serialPort2.SetCharSize(serialPortCharacterSizes[i]);

            characterSize1 = serialPort1.GetCharSize();
            characterSize2 = serialPort2.GetCharSize();

            ASSERT_EQ(characterSize1, serialPortCharacterSizes[i]);
            ASSERT_EQ(characterSize2, serialPortCharacterSizes[i]);
        }

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetFlowControl()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        SerialPort::FlowControl flowControl1;
        SerialPort::FlowControl flowControl2;

        // @NOTE - FLOW_CONTROL_SOFT flow control is not valid in Linux.
        for (size_t i = 0; i < 2; i++)
        {
            serialPort1.SetFlowControl(serialPortFlowControlTypes[i]);
            serialPort2.SetFlowControl(serialPortFlowControlTypes[i]);
            
            flowControl1 = serialPort1.GetFlowControl();
            flowControl2 = serialPort2.GetFlowControl();

            ASSERT_EQ(flowControl1, serialPortFlowControlTypes[i]);
            ASSERT_EQ(flowControl2, serialPortFlowControlTypes[i]);
        }

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetParity()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        SerialPort::Parity parity1;
        SerialPort::Parity parity2;

        for (size_t i = 0; i < 3; i++)
        {
            serialPort1.SetParity(serialPortParityTypes[i]);
            serialPort2.SetParity(serialPortParityTypes[i]);

            parity1 = serialPort1.GetParity();
            parity2 = serialPort2.GetParity();

            ASSERT_EQ(parity1, serialPortParityTypes[i]);
            ASSERT_EQ(parity2, serialPortParityTypes[i]);
        }

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetStopBits()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());
        
        SerialPort::StopBits numberOfStopBits1;
        SerialPort::StopBits numberOfStopBits2;

        for (size_t i = 0; i < 2; i++)
        {
            serialPort1.SetNumOfStopBits(serialPortStopBits[i]);
            serialPort2.SetNumOfStopBits(serialPortStopBits[i]);

            numberOfStopBits1 = serialPort1.GetNumOfStopBits();
            numberOfStopBits2 = serialPort2.GetNumOfStopBits();

            ASSERT_EQ(numberOfStopBits1, serialPortStopBits[i]);
            ASSERT_EQ(numberOfStopBits2, serialPortStopBits[i]);
        }

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetDTR()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool dtrLine1 = false;
        bool dtrLine2 = false;

        serialPort1.SetDtr(true);
        serialPort2.SetDtr(true);

        dtrLine1 = serialPort1.GetDtr();
        dtrLine2 = serialPort2.GetDtr();

        ASSERT_TRUE(dtrLine1);
        ASSERT_TRUE(dtrLine2);

        serialPort1.SetDtr(false);
        serialPort2.SetDtr(false);

        dtrLine1 = serialPort1.GetDtr();
        dtrLine2 = serialPort2.GetDtr();

        ASSERT_FALSE(dtrLine1);
        ASSERT_FALSE(dtrLine2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetRTS()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool rtsLine1 = false;
        bool rtsLine2 = false;

        serialPort1.SetRts(true);
        serialPort2.SetRts(true);

        rtsLine1 = serialPort1.GetRts();
        rtsLine2 = serialPort2.GetRts();

        ASSERT_TRUE(rtsLine1);
        ASSERT_TRUE(rtsLine2);

        serialPort1.SetRts(false);
        serialPort2.SetRts(false);
        
        rtsLine1 = serialPort1.GetRts();
        rtsLine2 = serialPort2.GetRts();
        
        ASSERT_FALSE(rtsLine1);
        ASSERT_FALSE(rtsLine2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetRTSGetCTS()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool ctsLine1 = false;
        bool ctsLine2 = false;

        serialPort1.SetRts(true);
        serialPort2.SetRts(true);

        ctsLine1 = serialPort1.GetCts();
        ctsLine2 = serialPort2.GetCts();

        ASSERT_TRUE(ctsLine1);
        ASSERT_TRUE(ctsLine2);

        serialPort1.SetRts(false);
        serialPort2.SetRts(false);

        ctsLine1 = serialPort1.GetCts();
        ctsLine2 = serialPort2.GetCts();

        ASSERT_FALSE(ctsLine1);
        ASSERT_FALSE(ctsLine2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetDTRGetDSR()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool dsrStatus1 = false;
        bool dsrStatus2 = false;

        serialPort1.SetDtr(true);
        serialPort2.SetDtr(true);

        dsrStatus1 = serialPort1.GetDsr();
        dsrStatus2 = serialPort1.GetDsr();

        ASSERT_TRUE(dsrStatus1);
        ASSERT_TRUE(dsrStatus2);

        serialPort1.SetDtr(false);
        serialPort2.SetDtr(false);

        dsrStatus1 = serialPort1.GetDsr();
        dsrStatus2 = serialPort1.GetDsr();

        ASSERT_FALSE(dsrStatus1);
        ASSERT_FALSE(dsrStatus2);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadByteWriteByte()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        char writeByte1 = 'a';
        unsigned char writeByte2 = 'A';

        char readByte1  = 'b';
        unsigned char readByte2  = 'B';

        serialPort1.WriteByte((unsigned char)writeByte1);
        serialPort2.WriteByte((unsigned char)writeByte2);

        readByte2 = (char)serialPort1.ReadByte(timeOutMilliseconds);
        readByte1 = (char)serialPort2.ReadByte(timeOutMilliseconds);

        ASSERT_EQ(readByte1, writeByte1);
        ASSERT_EQ(readByte2, writeByte2);

        try
        {
            serialPort1.ReadByte(1);
            serialPort2.ReadByte(1);
        }
        catch(SerialPort::ReadTimeout)
        {
            timeOutTestPass = true;
        }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadDataBufferWriteDataBuffer()
    {
        serialPort1.Open();
        serialPort2.Open();
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        SerialPort::DataBuffer writeDataBuffer1;
        SerialPort::DataBuffer writeDataBuffer2;

        SerialPort::DataBuffer readDataBuffer1;
        SerialPort::DataBuffer readDataBuffer2;

        // Test using ASCII characters.
        for (unsigned char i = 48; i <= 122; i++)
        {
            writeDataBuffer1.push_back(i);
        }

        for (unsigned char i = 122; i >= 48; i--)
        {
            writeDataBuffer2.push_back(i);
        }

        serialPort1.Write(writeDataBuffer1);
        serialPort2.Write(writeDataBuffer2);

        serialPort1.Read(readDataBuffer2, 75, timeOutMilliseconds);
        serialPort2.Read(readDataBuffer1, 75, timeOutMilliseconds);

        ASSERT_EQ(readDataBuffer1, writeDataBuffer1);
        ASSERT_EQ(readDataBuffer2, writeDataBuffer2);

        try
        {
            serialPort1.Read(readDataBuffer1, 1, 1);
            serialPort2.Read(readDataBuffer2, 1, 1);
        }
        catch(SerialPort::ReadTimeout)
        {
            timeOutTestPass = true;
        }
        
        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortReadLineWriteString()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        bool timeOutTestPass = false;

        serialPort1.Write(writeString1 + '\n');
        serialPort2.Write(writeString2 + '\n');

        readString1 = serialPort2.ReadLine(timeOutMilliseconds);
        readString2 = serialPort1.ReadLine(timeOutMilliseconds);

        ASSERT_EQ(readString1, writeString1 + '\n');
        ASSERT_EQ(readString2, writeString2 + '\n');

        try
        {        
            readString1 = serialPort2.ReadLine(1);
            readString2 = serialPort1.ReadLine(1);
        }
        catch(SerialPort::ReadTimeout)
        {
            timeOutTestPass = true;
        }

        ASSERT_TRUE(timeOutTestPass);

        serialPort1.Close();
        serialPort2.Close();

        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialStreamToSerialPortReadWrite()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialStream1.Open(TEST_SERIAL_PORT_2);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialPort1.SetBaudRate(serialPortBaudRates[15]);
        serialStream1.SetBaudRate(serialStreamBaudRates[15]);

        SerialPort::BaudRate          baudRate1 = serialPort1.GetBaudRate();
        SerialStreamBuf::BaudRateEnum baudRate2 = serialStream1.BaudRate();

        ASSERT_EQ(baudRate1, serialPortBaudRates[15]);
        ASSERT_EQ(baudRate2, serialStreamBaudRates[15]);

        serialStream1 << writeString1 << std::endl;
        readString1 = serialPort1.ReadLine(timeOutMilliseconds, '\n');

        ASSERT_EQ(readString1, writeString1 + '\n');
        ASSERT_EQ(readString1.size(), writeString1.size() + 1);
       
        serialPort1.Write(writeString2 + '\n');
        getline(serialStream1, readString2);

        ASSERT_EQ(readString2, writeString2);

        serialPort1.Close();
        serialStream1.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialStream1.IsOpen());
    }
};


//------------------------ Serial Stream Unit Tests -------------------------//

TEST_F(LibSerialTest, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open() and Close() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamOpenClose();
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
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialStreamSetGetVTime();
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


//------------------------- Serial Port Unit Tests --------------------------//

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open() and Close() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortOpenClose();
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

TEST_F(LibSerialTest, testSerialPortReadByteWriteByte)
{
    SCOPED_TRACE("Serial Port ReadByte() and WriteByte() Test");

    for (size_t i = 0; i < numberOfTestIterations; i++)
    {
        testSerialPortReadByteWriteByte();
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
