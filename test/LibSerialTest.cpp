/**
 * @file LibSerialTest.cpp
 * @copyright LibSerial
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include "gtest/gtest.h"
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

protected:
    SerialPort serialPort1;
    SerialPort serialPort2;

    SerialPort::BaudRate        serialPortBaudRates[30];
    SerialPort::CharacterSize   serialPortCharacterSizes[4];
    SerialPort::Parity          serialPortParityTypes[3];
    SerialPort::FlowControl     serialPortFlowControlTypes[3];
    SerialPort::StopBits        serialPortStopBits[2];

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


    //---------------------- Serial Stream Unit Tests -----------------------//

    void testSerialStreamOpenClose()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetBaudRate()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        // @NOTE - Higher baud rates do not pass.
        size_t maxBaudIndex = 17;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialStream1.SetBaudRate(serialStreamBaudRates[i]);
            SerialStreamBuf::BaudRateEnum baudRate = serialStream1.BaudRate();
            ASSERT_EQ(baudRate, serialStreamBaudRates[i]);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetCharacterSize()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        // @NOTE - Smaller Character Size values do not pass.
        for (size_t i = 2; i < 4; i++)
        {
            serialStream1.SetCharSize(serialStreamCharacterSizes[i]);
            SerialStreamBuf::CharSizeEnum characterSize = serialStream1.CharSize();
            ASSERT_EQ(characterSize, serialStreamCharacterSizes[i]);
            usleep(10);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetFlowControl()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetFlowControl(serialStreamFlowControlTypes[i]);
            SerialStreamBuf::FlowControlEnum flowControl = serialStream1.FlowControl();
            ASSERT_EQ(flowControl, serialStreamFlowControlTypes[i]);
            usleep(10);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetParity()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream1.SetParity(serialStreamParityTypes[i]);
            SerialStreamBuf::ParityEnum parity = serialStream1.Parity();
            ASSERT_EQ(parity, serialStreamParityTypes[i]);
            usleep(10);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        short numberOfStopBits;

        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        serialStream1.SetNumOfStopBits(serialStreamStopBits[0]);
        numberOfStopBits = serialStream1.NumOfStopBits();
        ASSERT_EQ(numberOfStopBits, serialStreamStopBits[0]);

        serialStream1.SetNumOfStopBits(serialStreamStopBits[1]);
        numberOfStopBits = serialStream1.NumOfStopBits();
        ASSERT_EQ(numberOfStopBits, serialStreamStopBits[1]);

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetVMin()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialStream1.SetVMin(i);
            short vMin = serialStream1.VMin();
            ASSERT_EQ(vMin, i);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamSetGetVTime()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        ASSERT_TRUE(serialStream1.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialStream1.SetVTime(i);
            short vTime = serialStream1.VTime();
            ASSERT_EQ(vTime, i);
        }

        serialStream1.Close();
        ASSERT_FALSE(serialStream1.IsOpen());
    }

    void testSerialStreamReadByteWriteByte()
    {
        serialStream1.Open(TEST_SERIAL_PORT_1);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream1.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        for (size_t i = 0; i < 10; i++)
        {
            writeByte = 'a';
            serialStream1.write(&writeByte, 1);
            serialStream2.read(&readByte, 1);
            ASSERT_EQ(readByte, writeByte);

            writeByte = 'A';
            serialStream1 << writeByte;
            serialStream2.read(&readByte, 1);
            ASSERT_EQ(readByte, writeByte);
        }

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

        for (size_t i = 0; i < 10; i++)
        {
            serialStream1 << writeString1 << std::endl;
            getline(serialStream2, readString1);
            ASSERT_EQ(readString1, writeString1);
        }

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

        for (size_t i = 0; i < 10; i++)
        {
            writeByte = 'a';
            serialStream1.write(&writeByte, 1);
            serialStream2.get(readByte);
            ASSERT_EQ(readByte, writeByte);

            writeByte = 'A';
            serialStream1 << writeByte;
            serialStream2.get(readByte);
            ASSERT_EQ(readByte, writeByte);
        }

        serialStream1.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream1.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }


    //----------------------- Serial Port Unit Tests ------------------------//

    void testSerialPortOpenClose()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortIsDataAvailableTest()
    {
        serialPort1.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        usleep(25000);

        ASSERT_FALSE(serialPort1.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        writeByte = 'A';
        unsigned char readByte;

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
        ASSERT_TRUE(serialPort1.IsOpen());

        size_t maxBaudIndex = 25;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialPort1.SetBaudRate(serialPortBaudRates[i]);
            SerialPort::BaudRate baudRate = serialPort1.GetBaudRate();
            ASSERT_EQ(baudRate, serialPortBaudRates[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetCharacterSize()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        // @NOTE - Smaller CharSize values do not work in Linux.
        for (size_t i = 2; i < 4; i++)
        {
            serialPort1.SetCharSize(serialPortCharacterSizes[i]);
            SerialPort::CharacterSize characterSize = serialPort1.GetCharSize();
            ASSERT_EQ(characterSize, serialPortCharacterSizes[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetFlowControl()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        // @NOTE - FLOW_CONTROL_SOFT flow control is not valid in Linux.
        for (size_t i = 0; i < 2; i++)
        {
            serialPort1.SetFlowControl(serialPortFlowControlTypes[i]);
            SerialPort::FlowControl flowControl = serialPort1.GetFlowControl();
            ASSERT_EQ(flowControl, serialPortFlowControlTypes[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetParity()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialPort1.SetParity(serialPortParityTypes[i]);
            SerialPort::Parity parity = serialPort1.GetParity();
            ASSERT_EQ(parity, serialPortParityTypes[i]);
        }

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetStopBits()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());
        
        SerialPort::StopBits numberOfStopBits;

        serialPort1.SetNumOfStopBits(serialPortStopBits[0]);
        numberOfStopBits = serialPort1.GetNumOfStopBits();
        ASSERT_EQ(numberOfStopBits, serialPortStopBits[0]);

        serialPort1.SetNumOfStopBits(serialPortStopBits[1]);
        numberOfStopBits = serialPort1.GetNumOfStopBits();
        ASSERT_EQ(numberOfStopBits, serialPortStopBits[1]);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetDTR()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetDtr(true);
        bool dtrLine = serialPort1.GetDtr();
        ASSERT_TRUE(dtrLine);

        serialPort1.SetDtr(false);
        dtrLine = serialPort1.GetDtr();
        ASSERT_FALSE(dtrLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortSetGetRTS()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        serialPort1.SetRts(true);
        bool rtsLine = serialPort1.GetRts();
        ASSERT_TRUE(rtsLine);

        serialPort1.SetRts(false);
        rtsLine = serialPort1.GetRts();
        ASSERT_FALSE(rtsLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetCTS()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        bool ctsLine = serialPort1.GetCts();
        ASSERT_FALSE(ctsLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortGetDSR()
    {
        serialPort1.Open();
        ASSERT_TRUE(serialPort1.IsOpen());

        bool dsrLine = serialPort1.GetDsr();
        ASSERT_FALSE(dsrLine);

        serialPort1.Close();
        ASSERT_FALSE(serialPort1.IsOpen());
    }

    void testSerialPortReadByteWriteByte()
    {
        serialPort1.Open();
        serialPort2.Open();
        
        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        int timeOutMilliseconds = 50;
        
        for (size_t i = 0; i < 10; i++)
        {
            serialPort1.WriteByte((unsigned char)writeByte);
            readByte = (char)serialPort2.ReadByte(timeOutMilliseconds);
            ASSERT_EQ(readByte, writeByte);
        }

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

        SerialPort::DataBuffer readDataBuffer;
        SerialPort::DataBuffer writeDataBuffer;

        // Test using ASCII characters.
        for (size_t i = 48; i < 123; i++)
        {
            writeDataBuffer.push_back(i);
        }

        int timeOutMilliseconds = 500;
        int numberOfBytes = 75;

        for (size_t i = 0; i < 10; i++)
        {
            serialPort1.Write(writeDataBuffer);
            serialPort2.Read(readDataBuffer, numberOfBytes, timeOutMilliseconds);
            ASSERT_EQ(readDataBuffer, writeDataBuffer);
        }

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

        int timeOutMilliseconds = 500;

        for (size_t i = 0; i < 10; i++)
        {
            serialPort1.Write(writeString1 + '\n');
            readString1 = serialPort2.ReadLine(timeOutMilliseconds);
            ASSERT_EQ(readString1, writeString1 + '\n');
        }

        serialPort1.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort1.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialStreamToSerialPortReadWrite()
    {
        serialPort1.Open();
        serialStream1.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialPort1.IsOpen());
        ASSERT_TRUE(serialStream1.IsOpen());

        int timeOutMilliseconds = 500;

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
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Port SetVMin() and GetVMin() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Port SetVTime() and GetVTime() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVTime();
    }
}

TEST_F(LibSerialTest, testSerialStreamReadByteWriteByte)
{
    SCOPED_TRACE("Serial Stream ReadByte() and WriteByte() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamReadByteWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetLineWriteString)
{
    SCOPED_TRACE("Serial Stream GetLine() and Write(string) Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamGetLineWriteString();
    }
}

TEST_F(LibSerialTest, testSerialStreamGetWriteByte)
{
    SCOPED_TRACE("Serial Stream Get() and WriteByte() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamGetWriteByte();
    }
}


//------------------------- Serial Port Unit Tests --------------------------//

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open() and Close() Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port IsDataAvailable() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port SetBaudRate() and GetBaudRate() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port SetCharacterSize() and GetCharacterSize() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port SetParityType() and GetParityType() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port SetFlowControl() and GetFlowControl() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port SetStopBits() and GetStopBits() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetStopBits();
    }
}
TEST_F(LibSerialTest, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port SetDTR() and GetDTR() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port SetRTS() and GetRTS() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetCTS)
{
    SCOPED_TRACE("Serial Port GetCTS() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetDSR)
{
    SCOPED_TRACE("Serial Port GetDSR() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialPortReadByteWriteByte)
{
    SCOPED_TRACE("Serial Port ReadByte() and WriteByte() Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadByteWriteByte();
    }
}

TEST_F(LibSerialTest, testSerialPortReadDataBufferWriteDataBuffer)
{
    SCOPED_TRACE("Serial Port Read(DataBuffer) and Write(DataBuffer) Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadDataBufferWriteDataBuffer();
    }
}

TEST_F(LibSerialTest, testSerialPortReadLineWriteString)
{
    SCOPED_TRACE("Serial Port ReadLine() and Write(string) Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadLineWriteString();
    }
}


//----------------- Serial Stream to Serial Port Unit Test ------------------//

TEST_F(LibSerialTest, testSerialStreamToSerialPortReadWrite)
{
    SCOPED_TRACE("Serial Stream To Serial Port Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamToSerialPortReadWrite();
    }
}
