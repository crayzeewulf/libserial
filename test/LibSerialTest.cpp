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

// Default Serial Port and Baud Rate.
#define TEST_SERIAL_PORT   "/dev/ttyUSB0"
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"

using namespace LibSerial;

class LibSerialTest
    : public ::testing::Test
{
public:

protected:

    virtual void SetUp()
    {
        writeString = "Quidquid latine dictum sit, altum sonatur. (Whatever is said in Latin sounds profound.)";

        serialStreamBaudRate[0]   =  SerialStreamBuf::BAUD_50;
        serialStreamBaudRate[1]   =  SerialStreamBuf::BAUD_75;
        serialStreamBaudRate[2]   =  SerialStreamBuf::BAUD_110;
        serialStreamBaudRate[3]   =  SerialStreamBuf::BAUD_134;
        serialStreamBaudRate[4]   =  SerialStreamBuf::BAUD_150;
        serialStreamBaudRate[5]   =  SerialStreamBuf::BAUD_200;
        serialStreamBaudRate[6]   =  SerialStreamBuf::BAUD_300;
        serialStreamBaudRate[7]   =  SerialStreamBuf::BAUD_600;
        serialStreamBaudRate[8]   =  SerialStreamBuf::BAUD_1200;
        serialStreamBaudRate[9]   =  SerialStreamBuf::BAUD_1800;
        serialStreamBaudRate[10]  =  SerialStreamBuf::BAUD_2400;
        serialStreamBaudRate[11]  =  SerialStreamBuf::BAUD_4800;
        serialStreamBaudRate[12]  =  SerialStreamBuf::BAUD_9600;
        serialStreamBaudRate[13]  =  SerialStreamBuf::BAUD_19200;
        serialStreamBaudRate[14]  =  SerialStreamBuf::BAUD_38400;
        serialStreamBaudRate[15]  =  SerialStreamBuf::BAUD_57600;
        serialStreamBaudRate[16]  =  SerialStreamBuf::BAUD_115200;
        serialStreamBaudRate[17]  =  SerialStreamBuf::BAUD_230400;
        serialStreamBaudRate[18]  =  SerialStreamBuf::BAUD_460800;
        serialStreamBaudRate[19]  =  SerialStreamBuf::BAUD_500000;
        serialStreamBaudRate[20]  =  SerialStreamBuf::BAUD_576000;
        serialStreamBaudRate[21]  =  SerialStreamBuf::BAUD_921600;
        serialStreamBaudRate[22]  =  SerialStreamBuf::BAUD_1000000;
        serialStreamBaudRate[23]  =  SerialStreamBuf::BAUD_1152000;
        serialStreamBaudRate[24]  =  SerialStreamBuf::BAUD_1500000;
        serialStreamBaudRate[25]  =  SerialStreamBuf::BAUD_2000000;
        serialStreamBaudRate[26]  =  SerialStreamBuf::BAUD_2500000;
        serialStreamBaudRate[27]  =  SerialStreamBuf::BAUD_3000000;
        serialStreamBaudRate[28]  =  SerialStreamBuf::BAUD_3500000;
        serialStreamBaudRate[29]  =  SerialStreamBuf::BAUD_4000000;

        serialStreamCharSize[0] = SerialStreamBuf::CHAR_SIZE_5;
        serialStreamCharSize[1] = SerialStreamBuf::CHAR_SIZE_6;
        serialStreamCharSize[2] = SerialStreamBuf::CHAR_SIZE_7;
        serialStreamCharSize[3] = SerialStreamBuf::CHAR_SIZE_8;

        serialStreamParity[0] = SerialStreamBuf::PARITY_EVEN;
        serialStreamParity[1] = SerialStreamBuf::PARITY_ODD;
        serialStreamParity[2] = SerialStreamBuf::PARITY_NONE;

        serialStreamFlowControl[0] = SerialStreamBuf::FLOW_CONTROL_HARD;
        serialStreamFlowControl[1] = SerialStreamBuf::FLOW_CONTROL_SOFT;
        serialStreamFlowControl[2] = SerialStreamBuf::FLOW_CONTROL_NONE;

        serialStreamNumOfStopBits[0] = 1;
        serialStreamNumOfStopBits[1] = 2;
    }

    void testSerialStreamOpenClose()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamReadAndWriteTest()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        serialStream << writeString;
        serialStream << "\n";

        getline(serialStream2, readString);
        ASSERT_EQ(readString, writeString);

        // @TODO - Each of these styles of read/write also need assertion tests created.
        // read()
        // get()
        // getline()
        // write()
        // Read(DataBuffer& dataBuffer)
        // ReadByte(unsigned int msTimeout)
        // ReadLine(unsigned int msTimeout, char lineTerminator)
        // Write(DataBuffer& dataBuffer)
        // Write(std::string& dataString)
        // WriteByte(unsigned char dataByte)

        serialStream.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamIsDataAvailableTest()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT_2);

        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        // serialStream << writeString;

        // bool dataAvailable = serialStream2.IsDataAvailable();
        // ASSERT_TRUE(dataAvailable);

        // serialStream2 >> readString;
        // dataAvailable = serialStream2.IsDataAvailable();
        // ASSERT_FALSE(dataAvailable);

        serialStream.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamSetGetBaudRate()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        size_t maxBaudIndex = 17;
        
        // #ifdef __linux__
        //     maxBaudIndex = 26;
            
        //     #if __MAX_BAUD > B2000000
        //         maxBaudIndex = 30;
        //     #endif
        
        // #endif

        for (size_t i = 0; i < maxBaudIndex; i++)
        {
            serialStream.SetBaudRate(serialStreamBaudRate[i]);
            SerialStreamBuf::BaudRateEnum baudRate = serialStream.BaudRate();
            ASSERT_EQ(baudRate, serialStreamBaudRate[i]);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetCharSize()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        // @TODO - Why don't the smaller CharSize values work?
        for (size_t i = 2; i < 4; i++)
        {
            serialStream.SetCharSize(serialStreamCharSize[i]);
            SerialStreamBuf::CharSizeEnum charSize = serialStream.CharSize();
            ASSERT_EQ(charSize, serialStreamCharSize[i]);
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
            serialStream.SetParity(serialStreamParity[i]);
            SerialStreamBuf::ParityEnum parity = serialStream.Parity();
            ASSERT_EQ(parity, serialStreamParity[i]);
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
            serialStream.SetFlowControl(serialStreamFlowControl[i]);
            SerialStreamBuf::FlowControlEnum flowControl = serialStream.FlowControl();
            ASSERT_EQ(flowControl, serialStreamFlowControl[i]);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        for (size_t i = 0; i < 2; i++)
        {
            serialStream.SetNumOfStopBits(serialStreamNumOfStopBits[i]);
            short numOfStopBits = serialStream.NumOfStopBits();
            ASSERT_EQ(numOfStopBits, serialStreamNumOfStopBits[i]);
        }

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetDTR()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        // serialStream.SetDtr(true);
        // bool dtrLine = serialStream.GetDtr();
        // ASSERT_TRUE(dtrLine);

        // serialStream.SetDtr(false);
        // dtrLine = serialStream.GetDtr();
        // ASSERT_FALSE(dtrLine);

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetRTS()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        // serialStream.SetRts(true);
        // bool rtsLine = serialStream.GetRts();
        // ASSERT_TRUE(rtsLine);

        // serialStream.SetRts(false);
        // rtsLine = serialStream.GetRts();
        // ASSERT_FALSE(rtsLine);

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamGetCTS()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        // serialStream.SetCts(true);
        // bool ctsLine = serialStream.GetCts();
        // ASSERT_TRUE(ctsLine);

        // serialStream.SetCts(false);
        // ctsLine = serialStream.GetCts();
        // ASSERT_FALSE(ctsLine);

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamGetDSR()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        // serialStream.SetDsr(true);
        // bool dsrLine = serialStream.GetDsr();
        // ASSERT_TRUE(dsrLine);

        // serialStream.SetDsr(false);
        // dsrLine = serialStream.GetDsr();
        // ASSERT_FALSE(dsrLine);

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    SerialStreamBuf::BaudRateEnum    serialStreamBaudRate[30];
    SerialStreamBuf::CharSizeEnum    serialStreamCharSize[4];
    SerialStreamBuf::ParityEnum      serialStreamParity[3];
    SerialStreamBuf::FlowControlEnum serialStreamFlowControl[3];

    short serialStreamNumOfStopBits[2];

    // std::string portName = "dev/ttyS0";
    // SerialPort seralPort("/dev/ttyS0");

    SerialStream serialStream;
    SerialStream serialStream2;

    std::string readString;
    std::string writeString;
};

TEST_F(LibSerialTest, testSerialStreamOpenClose)
{
    SCOPED_TRACE("Serial Stream Open and Close Test");
    testSerialStreamOpenClose();
}

TEST_F(LibSerialTest, testSerialStreamReadAndWriteTest)
{
    SCOPED_TRACE("Serial Stream Read and Write Test");
    testSerialStreamReadAndWriteTest();
}

TEST_F(LibSerialTest, testSerialStreamIsDataAvailableTest)
{
    SCOPED_TRACE("Serial Stream Is Data Available Test");
    testSerialStreamIsDataAvailableTest();
}

TEST_F(LibSerialTest, testSerialStreamSetGetBaudRate)
{
    SCOPED_TRACE("Serial Stream Set and Get Baud Rate Test");
    testSerialStreamSetGetBaudRate();
}

TEST_F(LibSerialTest, testSerialStreamSetGetCharSize)
{
    SCOPED_TRACE("Serial Stream Set and Get Char Size Test");
    testSerialStreamSetGetCharSize();
}

TEST_F(LibSerialTest, testSerialStreamSetGetParity)
{
    SCOPED_TRACE("Serial Stream Set and Get Parity Type Test");
    testSerialStreamSetGetParity();
}

TEST_F(LibSerialTest, testSerialStreamSetGetFlowControl)
{
    SCOPED_TRACE("Serial Stream Set and Get Flow Control Test");
    testSerialStreamSetGetFlowControl();
}

TEST_F(LibSerialTest, testSerialStreamSetGetStopBits)
{
    SCOPED_TRACE("Serial Stream Set and Get Stop Bits Test");
    testSerialStreamSetGetStopBits();
}

TEST_F(LibSerialTest, testSerialStreamSetGetDTR)
{
    SCOPED_TRACE("Serial Stream Set and Get Test");
    testSerialStreamSetGetDTR();
}

TEST_F(LibSerialTest, testSerialStreamSetGetRTS)
{
    SCOPED_TRACE("Serial Stream Set and Get Test");
    testSerialStreamSetGetRTS();
}

TEST_F(LibSerialTest, testSerialStreamGetCTS)
{
    SCOPED_TRACE("Serial Stream Set and Get CTS Test");
    testSerialStreamGetCTS();
}

TEST_F(LibSerialTest, testSerialStreamGetDSR)
{
    SCOPED_TRACE("Serial Stream Set and Get DSR Test");
    testSerialStreamGetDSR();
}
