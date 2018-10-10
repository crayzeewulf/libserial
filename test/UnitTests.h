/******************************************************************************
 *   @file UnitTest.h                                                         *
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

#include "SerialPort.h"
#include "SerialPortConstants.h"
#include "SerialStream.h"

#include <gtest/gtest.h>
#include <mutex>

/**
 * @brief Default Serial Port 1.
 */
#define TEST_SERIAL_PORT_1 "/dev/ttyUSB0"

/**
 * @brief Default Serial Port 2.
 */
#define TEST_SERIAL_PORT_2 "/dev/ttyUSB1"

/**
 * @namespace Libserial
 */
namespace LibSerial
{
    class LibSerialTest : public ::testing::Test
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit LibSerialTest();

        /**
         * @brief Default Destructor.
         */
        virtual ~LibSerialTest();

        /**
         * @param The number of iterations to perform on each unit test.
         */
        size_t numberOfTestIterations = 10;

    protected:

        /**
         * @brief Method for performing an required test initializations.
         */
        virtual void SetUp();

        /**
         * @brief Gets the time since epoch in milliseconds.
         * @return Returns the time since epoch in milliseconds
         */
        size_t getTimeInMilliSeconds();
        
        /**
         * @brief Gets the time since epoch in microseconds.
         * @return Returns the time since epoch in microseconds
         */
        size_t getTimeInMicroSeconds();

        //---------------------- Serial Stream Unit Tests -----------------------//

        /**
         * @brief Tests constructor overloads and destructors.
         */
        void testSerialStreamConstructors();

        /**
         * @brief Tests correct functionality for opening and closing serial streams.
         */
        void testSerialStreamOpenClose();

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcdrain().
         */
        void testSerialStreamDrainWriteBuffer();

        /**
         * @brief Tests correct functionality for draining the hardware input (read) buffer using tcflush();
         */
        void testSerialStreamFlushInputBuffer();

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcflush();
         */
        void testSerialStreamFlushOutputBuffer();

        /**
         * @brief Tests correct functionality for draining the hardware read/write buffers using tcflush();
         */
        void testSerialStreamFlushIOBuffers();

        /**
         * @brief Tests correct functionality for checking if data is available.
         */
        void testSerialStreamIsDataAvailableTest();

        /**
         * @brief Tests for correct functionality of the Open() and Close() methods.
         */
        void testSerialStreamIsOpenTest();

        /**
         * @brief Tests for correct functionality of the SetBaudRate() and GetBaudRate() methods.
         */
        void testSerialStreamSetGetBaudRate();

        /**
         * @brief Tests for correct functionality of the SetCharacterSize() and GetCharactersize() methods.
         */
        void testSerialStreamSetGetCharacterSize();

        /**
         * @brief Tests for correct functionality of the SetFlowControl() and GetFlowControl() methods.
         */
        void testSerialStreamSetGetFlowControl();

        /**
         * @brief Tests for correct functionality of the SetParity() and GetParity() methods.
         */
        void testSerialStreamSetGetParity();

        /**
         * @brief Tests for correct functionality of the SetStopBits() and GetStopBits() methods.
         */
        void testSerialStreamSetGetStopBits();

        /**
         * @brief Tests for correct functionality of the SetVMin() and GetVMin() methods.
         */
        void testSerialStreamSetGetVMin();

        /**
         * @brief Tests for correct functionality of the SetVTime() and GetVMin() methods.
         */
        void testSerialStreamSetGetVTime();

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDTR() methods.
         */
        void testSerialStreamSetGetDTR();

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetRTS() methods.
         */
        void testSerialStreamSetGetRTS();

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetCTS() methods.
         */
        void testSerialStreamSetRTSGetCTS();

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDSR() methods.
         */
        void testSerialStreamSetDTRGetDSR();

        /**
         * @brief Tests for correct functionality of the GetFileDescriptor() method.
         */
        void testSerialStreamGetFileDescriptor();

        /**
         * @brief Tests for correct functionality of the GetNumberOfBytesAvailable() method.
         */
        void testSerialStreamGetNumberOfBytesAvailable();

        /**
         * @brief Tests for correct functionality of the GetAvailableSerialPorts() method.
         */
        void testSerialStreamGetAvailableSerialPorts();

        /**
         * @brief Tests for correct functionality of the ReadByte() and WriteByte() methods.
         */
        void testSerialStreamReadByteWriteByte();

        /**
         * @brief Tests for correct functionality of the GetLine() and WriteString() methods.
         */
        void testSerialStreamGetLineWriteString();

        /**
         * @brief Tests for correct functionality of the Get() and WriteByte() methods.
         */
        void testSerialStreamGetWriteByte();


        //----------------------- Serial Port Unit Tests ------------------------//

        /**
         * @brief Tests constructor overloads and destructors.
         */
        void testSerialPortConstructors();

        /**
         * @brief Tests correct functionality for opening and closing serial ports.
         */
        void testSerialPortOpenClose();

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcdrain().
         */
        void testSerialPortDrainWriteBuffer();

        /**
         * @brief Tests correct functionality for draining the hardware input (read) buffer using tcflush();
         */
        void testSerialPortFlushInputBuffer();

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcflush();
         */
        void testSerialPortFlushOutputBuffer();

        /**
         * @brief Tests correct functionality for draining the hardware read/write buffers using tcflush();
         */
        void testSerialPortFlushIOBuffers();

        /**
         * @brief Tests correct functionality for checking if data is available.
         */
        void testSerialPortIsDataAvailableTest();

        /**
         * @brief Tests for correct functionality of the Open() and Close() methods.
         */
        void testSerialPortIsOpenTest();

        /**
         * @brief Tests for correct functionality of the SetBaudRate() and GetBaudRate() methods.
         */
        void testSerialPortSetGetBaudRate();

        /**
         * @brief Tests for correct functionality of the SetCharacterSize() and GetCharactersize() methods.
         */
        void testSerialPortSetGetCharacterSize();

        /**
         * @brief Tests for correct functionality of the SetFlowControl() and GetFlowControl() methods.
         */
        void testSerialPortSetGetFlowControl();

        /**
         * @brief Tests for correct functionality of the SetParity() and GetParity() methods.
         */
        void testSerialPortSetGetParity();

        /**
         * @brief Tests for correct functionality of the SetStopBits() and GetStopBits() methods.
         */
        void testSerialPortSetGetStopBits();

        /**
         * @brief Tests for correct functionality of the SetVMin() and GetVMin() methods.
         */
        void testSerialPortSetGetVMin();

        /**
         * @brief Tests for correct functionality of the SetVTime() and GetVMin() methods.
         */
        void testSerialPortSetGetVTime();

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDTR() methods.
         */
        void testSerialPortSetGetDTR();

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetRTS() methods.
         */
        void testSerialPortSetGetRTS();

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetCTS() methods.
         */
        void testSerialPortSetRTSGetCTS();

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDSR() methods.
         */
        void testSerialPortSetDTRGetDSR();

        /**
         * @brief Tests for correct functionality of the GetFileDescriptor() method.
         */
        void testSerialPortGetFileDescriptor();

        /**
         * @brief Tests for correct functionality of the GetNumberOfBytesAvailable() method.
         */
        void testSerialPortGetNumberOfBytesAvailable();

        /**
         * @brief Tests for correct functionality of the GetAvailableSerialPorts() method.
         */
        void testSerialPortGetAvailableSerialPorts();

        /**
         * @brief Tests for correct functionality of the ReadDataBuffer() and WriteDataBuffer() methods.
         */
        void testSerialPortReadDataBufferWriteDataBuffer();

        /**
         * @brief Tests for correct functionality of the ReadString() and WriteString() methods.
         */
        void testSerialPortReadStringWriteString();

        /**
         * @brief Tests for correct functionality of the ReadByte() and WriteByte() methods.
         */
        void testSerialPortReadByteWriteByte();

        /**
         * @brief Tests for correct functionality of the ReadLine() and WriteString() methods.
         */
        void testSerialPortReadLineWriteString();

        /**
         * @brief Tests for correct functionality for writing data from a serial stream object and reading that data from a serial port object..
         */
        void testSerialStreamToSerialPortReadWrite();

        //--------------------- Multi-Thread Unit Tests ---------------------//

        /**
         * @brief Tests for correct functionality of a multi-threaded serial stream application.
         */
        void serialStream1ThreadLoop();

        /**
         * @brief Tests for correct functionality of a multi-threaded serial port application.
         */
        void serialStream2ThreadLoop();

        /**
         * @brief Main loop for the multi-threaded serial stream unit test.
         */
        void serialPort1ThreadLoop();

        /**
         * @brief Main loop for the multi-threaded serial port unit test.
         */
        void serialPort2ThreadLoop();

        /**
         * @brief Entry point for the multi-thread serial stream unit test.
         */
        void testMultiThreadSerialStreamReadWrite();

        /**
         * @brief Entry point for the multi-thread serial port unit test.
         */
        void testMultiThreadSerialPortReadWrite();

        //---------------------- Unit Tests Parameters ----------------------//

        /**
         * @param C++11 thread std::mutex for locking parameters in the threaded unit tests.
         */
        std::mutex mutex;

        /**
         * @param Failure rate of serial communications being tracked in the threaded tests.
         */
        size_t failureRate;

        /**
         * @param Loop count variable.
         */
        size_t loopCount;

        /**
         * @param Timeout to be used for test methods.
         */
        size_t timeOutMilliseconds;

        /**
         * @param Time to allow the hardware read buffer to fill or empty.
         */
        unsigned int readBufferDelay;

        /**
         * @struct Standard baud rates.
         */
        std::vector<LibSerial::BaudRate> baudRates;

        /**
         * @struct Standard character sizes.
         */
        std::vector<LibSerial::CharacterSize> characterSizes;

        /**
         * @struct Standard flow control types.
         */
        std::vector<LibSerial::FlowControl> flowControlTypes;

        /**
         * @struct Standard flow parity types.
         */
        std::vector<LibSerial::Parity> parityTypes;

        /**
         * @struct Standard number of stop bits.
         */
        std::vector<LibSerial::StopBits> stopBits;

        /**
         * @param Serial Stream instance 1 for unit testing applications.
         */
        LibSerial::SerialStream serialStream1;

        /**
         * @param Serial Stream instance 2 for unit testing applications.
         */
        LibSerial::SerialStream serialStream2;

        /**
         * @param Serial Port instance 1 for unit testing applications.
         */
        LibSerial::SerialPort serialPort1;

        /**
         * @param Serial Port instance 2 for unit testing applications.
         */
        LibSerial::SerialPort serialPort2;

        /**
         * @param String to store received data.
         */
        std::string readString1;

        /**
         * @param String to store received data.
         */
        std::string readString2;

        /**
         * @param String to store data to be written to the serial port.
         */
        std::string writeString1;

        /**
         * @param String to store data to be written to the serial port.
         */
        std::string writeString2;

    };
}
