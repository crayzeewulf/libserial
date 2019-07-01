/******************************************************************************
 * @file SerialPortUnitTests.h                                                *
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

#pragma once

#include "UnitTests.h"
#include "libserial/SerialPort.h"
#include "libserial/SerialPortConstants.h"

#include <gtest/gtest.h>
#include <mutex>

/**
 * @namespace Libserial
 */
namespace LibSerial
{
    class SerialPortUnitTests : public UnitTests
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit SerialPortUnitTests() ;

        /**
         * @brief Default Destructor.
         */
        virtual ~SerialPortUnitTests() ;

    protected:

        /**
         * @brief Tests constructor overloads and destructors.
         */
        void testSerialPortConstructors() ;

        /**
         * @brief Tests correct functionality for opening and closing serial ports.
         */
        void testSerialPortOpenClose() ;

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcdrain().
         */
        void testSerialPortDrainWriteBuffer() ;

        /**
         * @brief Tests correct functionality for draining the hardware input (read) buffer using tcflush() ;
         */
        void testSerialPortFlushInputBuffer() ;

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcflush() ;
         */
        void testSerialPortFlushOutputBuffer() ;

        /**
         * @brief Tests correct functionality for draining the hardware read/write buffers using tcflush() ;
         */
        void testSerialPortFlushIOBuffers() ;

        /**
         * @brief Tests correct functionality for checking if data is available.
         */
        void testSerialPortIsDataAvailableTest() ;

        /**
         * @brief Tests for correct functionality of the Open() and Close() methods.
         */
        void testSerialPortIsOpenTest() ;

        /**
         * @brief Tests for correct functionality of the SetBaudRate() and GetBaudRate() methods.
         */
        void testSerialPortSetGetBaudRate() ;

        /**
         * @brief Tests for correct functionality of the SetCharacterSize() and GetCharactersize() methods.
         */
        void testSerialPortSetGetCharacterSize() ;

        /**
         * @brief Tests for correct functionality of the SetFlowControl() and GetFlowControl() methods.
         */
        void testSerialPortSetGetFlowControl() ;

        /**
         * @brief Tests for correct functionality of the SetParity() and GetParity() methods.
         */
        void testSerialPortSetGetParity() ;

        /**
         * @brief Tests for correct functionality of the SetStopBits() and GetStopBits() methods.
         */
        void testSerialPortSetGetStopBits() ;

        /**
         * @brief Tests for correct functionality of the SetVMin() and GetVMin() methods.
         */
        void testSerialPortSetGetVMin() ;

        /**
         * @brief Tests for correct functionality of the SetVTime() and GetVMin() methods.
         */
        void testSerialPortSetGetVTime() ;

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDTR() methods.
         */
        void testSerialPortSetGetDTR() ;

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetRTS() methods.
         */
        void testSerialPortSetGetRTS() ;

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetCTS() methods.
         */
        void testSerialPortSetRTSGetCTS() ;

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDSR() methods.
         */
        void testSerialPortSetDTRGetDSR() ;

        /**
         * @brief Tests for correct functionality of the GetFileDescriptor() method.
         */
        void testSerialPortGetFileDescriptor() ;

        /**
         * @brief Tests for correct functionality of the GetNumberOfBytesAvailable() method.
         */
        void testSerialPortGetNumberOfBytesAvailable() ;

#ifdef __linux__
        /**
         * @brief Tests for correct functionality of the GetAvailableSerialPorts() method.
         */
        void testSerialPortGetAvailableSerialPorts() ;
#endif

        /**
         * @brief Tests for correct functionality of the ReadDataBuffer() and WriteDataBuffer() methods.
         */
        void testSerialPortReadDataBufferWriteDataBuffer() ;

        /**
         * @brief Tests for correct functionality of the ReadString() and WriteString() methods.
         */
        void testSerialPortReadStringWriteString() ;

        /**
         * @brief Tests for correct functionality of the ReadByte() and WriteByte() methods.
         */
        void testSerialPortReadByteWriteByte() ;

        /**
         * @brief Tests for correct functionality of the ReadLine() and WriteString() methods.
         */
        void testSerialPortReadLineWriteString() ;

    } ; // class SerialPortUnitTests

} // namespace LibSerial
