/**
 * @file LibSerialTest.cpp
 * @copyright LibSerial
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <SerialPort.h>
#include <SerialStream.h>

// Default Serial Port and Baud Rate.
#define TEST_SERIAL_PORT   "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"

using namespace LibSerial;

class LibSerialTest
    : public ::testing::Test
{
public:
    LibSerialTest() : serialPort(TEST_SERIAL_PORT), serialPort2(TEST_SERIAL_PORT_2) {} 

protected:
    SerialPort serialPort;
    SerialPort serialPort2;


    virtual void SetUp()
    {
        writeString = "Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)";

        baudRates[0]   =  BaudRate::BAUD_50;
        baudRates[1]   =  BaudRate::BAUD_75;
        baudRates[2]   =  BaudRate::BAUD_110;
        baudRates[3]   =  BaudRate::BAUD_134;
        baudRates[4]   =  BaudRate::BAUD_150;
        baudRates[5]   =  BaudRate::BAUD_200;
        baudRates[6]   =  BaudRate::BAUD_300;
        baudRates[7]   =  BaudRate::BAUD_600;
        baudRates[8]   =  BaudRate::BAUD_1200;
        baudRates[9]   =  BaudRate::BAUD_1800;
        baudRates[10]  =  BaudRate::BAUD_2400;
        baudRates[11]  =  BaudRate::BAUD_4800;
        baudRates[12]  =  BaudRate::BAUD_9600;
        baudRates[13]  =  BaudRate::BAUD_19200;
        baudRates[14]  =  BaudRate::BAUD_38400;
        baudRates[15]  =  BaudRate::BAUD_57600;
        baudRates[16]  =  BaudRate::BAUD_115200;
        baudRates[17]  =  BaudRate::BAUD_230400;
        baudRates[18]  =  BaudRate::BAUD_460800;
        baudRates[19]  =  BaudRate::BAUD_921600;
        baudRates[20]  =  BaudRate::BAUD_1000000;
        baudRates[21]  =  BaudRate::BAUD_1500000;
        baudRates[22]  =  BaudRate::BAUD_2000000;
        baudRates[23]  =  BaudRate::BAUD_2500000;
        baudRates[24]  =  BaudRate::BAUD_3000000;

        characterSizes[0] = CharacterSize::CHAR_SIZE_5;
        characterSizes[1] = CharacterSize::CHAR_SIZE_6;
        characterSizes[2] = CharacterSize::CHAR_SIZE_7;
        characterSizes[3] = CharacterSize::CHAR_SIZE_8;

        flowControlTypes[0] = FlowControl::FLOW_CONTROL_NONE;
        flowControlTypes[1] = FlowControl::FLOW_CONTROL_HARDWARE;
        flowControlTypes[2] = FlowControl::FLOW_CONTROL_SOFTWARE;

        parityTypes[0] = Parity::PARITY_EVEN;
        parityTypes[1] = Parity::PARITY_ODD;
        parityTypes[2] = Parity::PARITY_NONE;

        stopBits[0] = StopBits::STOP_BITS_1;
        stopBits[1] = StopBits::STOP_BITS_2;
    }


    //---------------------- Serial Stream Unit Tests -----------------------//

    void testSerialStreamOpenClose()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamReadWrite()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream.SetBaudRate(baudRates[16]);
        serialStream2.SetBaudRate(baudRates[16]);
        
        BaudRate baudRate = serialStream.GetBaudRate();
        BaudRate baudRate2 = serialStream2.GetBaudRate();
        
        ASSERT_EQ(baudRate, baudRates[16]);
        ASSERT_EQ(baudRate2, baudRates[16]);

        serialStream << writeString << std::endl;
        getline(serialStream2, readString);
        ASSERT_EQ(readString, writeString);

        serialStream.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());

        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        serialStream2 << writeString << std::endl;
        getline(serialStream, readString);
        ASSERT_EQ(readString, writeString);

        writeByte = 'a';
        
        serialStream.write(&writeByte, 1);
        serialStream << writeByte << std::endl;

        serialStream2.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialStream2.write(&writeByte, 1);
        serialStream2 << writeByte << std::endl;
        
        serialStream.read(&readByte, 1);
        ASSERT_EQ(readByte, writeByte);

        serialStream.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());

        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());        

        writeByte = 'A';
        
        serialStream.write(&writeByte, 1);
        serialStream << writeByte << std::endl;
        
        serialStream2.get(readByte);
        ASSERT_EQ(readByte, writeByte);

        serialStream2.write(&writeByte, 1);
        serialStream2 << writeByte << std::endl;
        
        serialStream.get(readByte);
        ASSERT_EQ(readByte, writeByte);

        serialStream.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetBaudRate()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        size_t maxBaudIndex = 25;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialStream.SetBaudRate(baudRates[i]);
            BaudRate baudRate = serialStream.GetBaudRate();
            ASSERT_EQ(baudRate, baudRates[i]);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetCharacterSize()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        // @TODO - Why don't the smaller Character Size values work?
        for (size_t i = 2; i < 4; i++)
        {
            serialStream.SetCharacterSize(characterSizes[i]);
            CharacterSize characterSize = serialStream.GetCharacterSize();
            ASSERT_EQ(characterSize, characterSizes[i]);
            usleep(10);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetFlowControl()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream.SetFlowControl(flowControlTypes[i]);
            FlowControl flowControl = serialStream.GetFlowControl();
            ASSERT_EQ(flowControl, flowControlTypes[i]);
            usleep(10);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetParity()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialStream.SetParity(parityTypes[i]);
            Parity parity = serialStream.GetParity();
            ASSERT_EQ(parity, parityTypes[i]);
            usleep(10);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        StopBits numberOfStopBits;

        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.SetNumberOfStopBits(stopBits[0]);
        numberOfStopBits = serialStream.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[0]);

        serialStream.SetNumberOfStopBits(stopBits[1]);
        numberOfStopBits = serialStream.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[1]);

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetVMin()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialStream.SetVMin(i);
            short vMin = serialStream.GetVMin();
            ASSERT_EQ(vMin, i);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetVTime()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialStream.SetVTime(i);
            short vTime = serialStream.GetVTime();
            ASSERT_EQ(vTime, i);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }


    //----------------------- Serial Port Unit Tests ------------------------//

    void testSerialPortOpenClose()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortReadWrite(const int timeOutMilliseconds)
    {
        serialPort.Open();
        serialPort2.Open();
        
        ASSERT_TRUE(serialPort.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        serialPort.SetBaudRate(baudRates[16]);
        serialPort2.SetBaudRate(baudRates[16]);
        
        BaudRate baudRate = serialPort.GetBaudRate();
        BaudRate baudRate2 = serialPort2.GetBaudRate();
        
        ASSERT_EQ(baudRate, baudRates[16]);
        ASSERT_EQ(baudRate2, baudRates[16]);

        SerialPort::DataBuffer readDataBuffer;
        SerialPort::DataBuffer writeDataBuffer;

        // Test using ASCII characters.
        for (size_t i = 48; i < 122; i++)
        {
            writeDataBuffer.push_back(i);
        }

        serialPort.Write(writeDataBuffer);
        tcdrain(serialPort.GetFileDescriptor());
        
        bytesRead = serialPort2.Read(readDataBuffer, 74, timeOutMilliseconds);

        ASSERT_EQ(readDataBuffer, writeDataBuffer);
        ASSERT_EQ(bytesRead, writeDataBuffer.size());

        unsigned char writeByte;
        unsigned char readByte;
        
        serialPort.WriteByte(writeByte);
        tcdrain(serialPort.GetFileDescriptor());

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());
        
        bytesRead = serialPort.ReadByte(readByte, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);
        ASSERT_EQ(bytesRead, 1);

        bytesRead = serialPort2.ReadByte(readByte, timeOutMilliseconds);
        ASSERT_EQ(readByte, writeByte);
        ASSERT_EQ(bytesRead, 1);

        serialPort.Write(writeString + '\n');
        tcdrain(serialPort.GetFileDescriptor());

        serialPort2.Write(writeString + '\n');
        tcdrain(serialPort2.GetFileDescriptor());

        bytesRead = serialPort.ReadLine(readString, '\n', timeOutMilliseconds);
        ASSERT_EQ(readString, writeString + '\n');
        ASSERT_EQ(bytesRead, writeString.size() + 1);
        
        bytesRead = serialPort2.ReadLine(readString, '\n', timeOutMilliseconds);
        ASSERT_EQ(readString, writeString + '\n');
        ASSERT_EQ(bytesRead, writeString.size() + 1);
        
        serialPort.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortIsDataAvailableTest()
    {
        serialPort.Open();
        serialPort2.Open();

        ASSERT_TRUE(serialPort.IsOpen());
        ASSERT_TRUE(serialPort2.IsOpen());

        ASSERT_FALSE(serialPort.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        unsigned char writeByte;
        unsigned char readByte;

        writeByte = 'A';

        serialPort.WriteByte(writeByte);
        tcdrain(serialPort.GetFileDescriptor());
        
        usleep(2000);

        ASSERT_TRUE(serialPort2.IsDataAvailable());

        bytesRead = serialPort2.ReadByte(readByte, 1);
        ASSERT_TRUE(bytesRead == 1);

        serialPort2.WriteByte(writeByte);
        tcdrain(serialPort2.GetFileDescriptor());
                
        usleep(2000);

        ASSERT_TRUE(serialPort.IsDataAvailable());
        
        bytesRead = serialPort.ReadByte(readByte, 1);
        ASSERT_TRUE(bytesRead == 1);

        ASSERT_FALSE(serialPort.IsDataAvailable());
        ASSERT_FALSE(serialPort2.IsDataAvailable());

        serialPort.Close();
        serialPort2.Close();
        
        ASSERT_FALSE(serialPort.IsOpen());
        ASSERT_FALSE(serialPort2.IsOpen());
    }

    void testSerialPortSetGetBaudRate()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        size_t maxBaudIndex = 25;

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialPort.SetBaudRate(baudRates[i]);
            BaudRate baudRate = serialPort.GetBaudRate();
            ASSERT_EQ(baudRate, baudRates[i]);
        }

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetCharacterSize()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        // @TODO - Why don't the smaller CharSize values work?
        for (size_t i = 2; i < 4; i++)
        {
            serialPort.SetCharacterSize(characterSizes[i]);
            CharacterSize characterSize = serialPort.GetCharacterSize();
            ASSERT_EQ(characterSize, characterSizes[i]);
        }

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetFlowControl()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        // @TODO - FLOW_CONTROL_SOFT flow control is not valid.
        for (size_t i = 0; i < 2; i++)
        {
            serialPort.SetFlowControl(flowControlTypes[i]);
            FlowControl flowControl = serialPort.GetFlowControl();
            ASSERT_EQ(flowControl, flowControlTypes[i]);
        }

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetParity()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        for (size_t i = 0; i < 3; i++)
        {
            serialPort.SetParity(parityTypes[i]);
            Parity parity = serialPort.GetParity();
            ASSERT_EQ(parity, parityTypes[i]);
        }

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetStopBits()
    {
        StopBits numberOfStopBits;

        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        serialPort.SetNumberOfStopBits(stopBits[0]);
        numberOfStopBits = serialPort.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[0]);

        serialPort.SetNumberOfStopBits(stopBits[1]);
        numberOfStopBits = serialPort.GetNumberOfStopBits();
        ASSERT_EQ(numberOfStopBits, stopBits[1]);

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetDTR()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        serialPort.SetDtr(true);
        bool dtrLine = serialPort.GetDtr();
        ASSERT_TRUE(dtrLine);

        serialPort.SetDtr(false);
        dtrLine = serialPort.GetDtr();
        ASSERT_FALSE(dtrLine);

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetRTS()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        serialPort.SetRts(true);
        bool rtsLine = serialPort.GetRts();
        ASSERT_TRUE(rtsLine);

        serialPort.SetRts(false);
        rtsLine = serialPort.GetRts();
        ASSERT_FALSE(rtsLine);

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortGetCTS()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        bool ctsLine = serialPort.GetCts();
        ASSERT_FALSE(ctsLine);

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortGetDSR()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        bool dsrLine = serialPort.GetDsr();
        ASSERT_FALSE(dsrLine);

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetVMin()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialPort.SetVMin(i);
            short vMin = serialPort.GetVMin();
            ASSERT_EQ(vMin, i);
        }

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialPortSetGetVTime()
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        for (short i = 0; i < 5; i++)
        {
            serialPort.SetVTime(i);
            short vTime = serialPort.GetVTime();
            ASSERT_EQ(vTime, i);
        }

        serialPort.Close();
        ASSERT_FALSE(serialPort.IsOpen());
    }

    void testSerialStreamToSerialPortReadWrite(const int timeOutMilliseconds)
    {
        serialPort.Open();
        ASSERT_TRUE(serialPort.IsOpen());

        serialStream.Open(TEST_SERIAL_PORT_2);
        ASSERT_TRUE(serialStream.IsOpen());

        serialPort.SetBaudRate(baudRates[16]);
        serialStream.SetBaudRate(baudRates[16]);
        
        BaudRate baudRate = serialPort.GetBaudRate();
        BaudRate baudRate2 = serialStream.GetBaudRate();
        
        ASSERT_EQ(baudRate, baudRates[16]);
        ASSERT_EQ(baudRate2, baudRates[16]);

        serialStream << writeString << std::endl;
        bytesRead = serialPort.ReadLine(readString, '\n', timeOutMilliseconds);
        ASSERT_EQ(readString, writeString + '\n');
        ASSERT_EQ(bytesRead, writeString.size() + 1);
       
        serialPort.Write(writeString + '\n');
        tcdrain(serialPort.GetFileDescriptor());
        getline(serialStream, readString);
        ASSERT_EQ(readString, writeString);

        serialPort.Close();
        serialStream.Close();
        
        ASSERT_FALSE(serialPort.IsOpen());
        ASSERT_FALSE(serialStream.IsOpen());
    }


    BaudRate        baudRates[25];
    CharacterSize   characterSizes[4];
    FlowControl     flowControlTypes[3];
    Parity          parityTypes[3];
    StopBits        stopBits[2];

    SerialStream serialStream;
    SerialStream serialStream2;

    std::string readString;
    std::string writeString;

    char writeByte;
    char readByte;

    int bytesRead = 0;
};



//------------------------ Serial Stream Unit Tests -------------------------//

TEST_F(LibSerialTest, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open and Close Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialStreamReadWrite)
{
    SCOPED_TRACE("Serial Stream Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamReadWrite();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream Set and Get Baud Rate Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Stream Set and Get Character Size Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream Set and Get Parity Type Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream Set and Get Flow Control Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream Set and Get Stop Bits Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetStopBits();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVMin)
{
    SCOPED_TRACE("Serial Port Set and Get vMin Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialStreamSetGetVTime)
{
    SCOPED_TRACE("Serial Port Set and Get vTime Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamSetGetVTime();
    }
}


//------------------------- Serial Port Unit Tests --------------------------//

TEST_F(LibSerialTest, testSerialPortOpenClose)
{
    SCOPED_TRACE("Serial Port Open and Close Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortOpenClose();
    }
}

TEST_F(LibSerialTest, testSerialPortReadWrite)
{
    int timeOutMilliseconds = 25;

    SCOPED_TRACE("Serial Port Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortReadWrite(timeOutMilliseconds);
    }
}

TEST_F(LibSerialTest, testSerialPortIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Port Is Data Available Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortIsDataAvailableTest();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetBaudRate)
{
    SCOPED_TRACE("Serial Port Set and Get Baud Rate Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetBaudRate();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetCharacterSize)
{
    SCOPED_TRACE("Serial Port Set and Get Character Size Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetCharacterSize();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetParity)
{
    SCOPED_TRACE("Serial Port Set and Get Parity Type Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetParity();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetFlowControl)
{
    SCOPED_TRACE("Serial Port Set and Get Flow Control Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetFlowControl();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetStopBits)
{
    SCOPED_TRACE("Serial Port Set and Get Stop Bits Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetStopBits();
    }
}
TEST_F(LibSerialTest, testSerialPortSetGetDTR)
{
    SCOPED_TRACE("Serial Port Set and Get DTR Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetDTR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetRTS)
{
    SCOPED_TRACE("Serial Port Set and Get RTS Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetRTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetCTS)
{
    SCOPED_TRACE("Serial Port Get CTS Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetCTS();
    }
}

TEST_F(LibSerialTest, testSerialPortGetDSR)
{
    SCOPED_TRACE("Serial Port Get DSR Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortGetDSR();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVMin)
{
    SCOPED_TRACE("Serial Port Set and Get vMin Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetVMin();
    }
}

TEST_F(LibSerialTest, testSerialPortSetGetVTime)
{
    SCOPED_TRACE("Serial Port Set and Get vTime Test");
    
    for (size_t i = 0; i < 100; i++)
    {
        testSerialPortSetGetVTime();
    }
}


//----------------- Serial Stream to Serial Port Unit Tests -----------------//

TEST_F(LibSerialTest, testSerialStreamToSerialPortReadWrite)
{
    int timeOutMilliseconds = 25;

    SCOPED_TRACE("Serial Stream To Serial Port Read and Write Test");

    for (size_t i = 0; i < 100; i++)
    {
        testSerialStreamToSerialPortReadWrite(timeOutMilliseconds);
    }
}
