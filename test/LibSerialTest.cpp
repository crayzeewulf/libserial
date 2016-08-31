/**
 * @file LibSerialTest.cpp
 * @copyright LibSerial
 */

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>

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

        serialStreamBaudRate    = SerialStreamBuf::BAUD_230400;
        serialStreamCharSize    = SerialStreamBuf::CHAR_SIZE_8;
        serialStreamParity      = SerialStreamBuf::PARITY_NONE;
        serialStreamFlowControl = SerialStreamBuf::FLOW_CONTROL_NONE;

        serialStreamNumOfStopBits = 2;
    }

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

        serialStream << writeString;
        serialStream2 >> readString;

        ASSERT_EQ(readString, writeString);

        serialStream.Close();
        serialStream2.Close();

        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamReadTest()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT);
        
        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());

        // serialStream << writeStringWithSpaces;
        // serialStream2 >> readString;

        // ReadLine()
        // ReadByte()
        // Read()

        serialStream.Close();
        serialStream2.Close();
        
        ASSERT_FALSE(serialStream.IsOpen());
        ASSERT_FALSE(serialStream2.IsOpen());
    }

    void testSerialStreamWriteTest()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        serialStream2.Open(TEST_SERIAL_PORT);
        
        ASSERT_TRUE(serialStream.IsOpen());
        ASSERT_TRUE(serialStream2.IsOpen());
        
        // Write(dataBuffer)
        // Write(std::string)
        // WriteByte()

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

        serialStream.SetBaudRate(serialStreamBaudRate);

        SerialStreamBuf::BaudRateEnum baudRate = serialStream.BaudRate();

        ASSERT_EQ(baudRate, serialStreamBaudRate);

        std::cout << "\tbaudRate = " << baudRate << std::endl;

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetCharSize()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.SetCharSize(serialStreamCharSize);

        SerialStreamBuf::CharSizeEnum charSize = serialStream.CharSize();

        ASSERT_EQ(charSize, serialStreamCharSize);

        std::cout << "\tcharSize = " << charSize << std::endl;

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetParity()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.SetParity(serialStreamParity);

        SerialStreamBuf::ParityEnum parity = serialStream.Parity();

        ASSERT_EQ(parity, serialStreamParity);

        std::cout << "\tparity = " << parity << std::endl;

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetFlowControl()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.SetFlowControl(serialStreamFlowControl);

        SerialStreamBuf::FlowControlEnum flowControl = serialStream.FlowControl();

        ASSERT_EQ(flowControl, serialStreamFlowControl);

        std::cout << "\tflowControl = " << flowControl << std::endl;

        serialStream.Close();
        ASSERT_FALSE(serialStream.IsOpen());
    }

    void testSerialStreamSetGetStopBits()
    {
        serialStream.Open(TEST_SERIAL_PORT);
        ASSERT_TRUE(serialStream.IsOpen());

        serialStream.SetNumOfStopBits(serialStreamNumOfStopBits);

        short numOfStopBits = serialStream.NumOfStopBits();

        ASSERT_EQ(numOfStopBits, serialStreamNumOfStopBits);

        std::cout << "\tnumOfStopBits = " << numOfStopBits << std::endl;

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

    SerialStreamBuf::BaudRateEnum    serialStreamBaudRate;
    SerialStreamBuf::CharSizeEnum    serialStreamCharSize;
    SerialStreamBuf::ParityEnum      serialStreamParity;
    SerialStreamBuf::FlowControlEnum serialStreamFlowControl;

    short serialStreamNumOfStopBits;

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

TEST_F(LibSerialTest, testSerialStreamReadWrite)
{
    SCOPED_TRACE("Serial Stream Read and Write Test");
    testSerialStreamReadWrite();
}

TEST_F(LibSerialTest, testSerialStreamReadTest)
{
    SCOPED_TRACE("Serial Stream Read Test");
    testSerialStreamReadTest();
}

TEST_F(LibSerialTest, testSerialStreamWriteTest)
{
    SCOPED_TRACE("Serial Stream Write Test");
    testSerialStreamWriteTest();
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
