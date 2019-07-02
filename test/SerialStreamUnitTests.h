/******************************************************************************
 * @file SerialStreamUnitTests.h                                              *
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
#include "libserial/SerialStream.h"

#include <gtest/gtest.h>
#include <mutex>

/**
 * @namespace Libserial
 */
namespace LibSerial
{
    class SerialStreamUnitTests : public UnitTests
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit SerialStreamUnitTests() ;

        /**
         * @brief Default Destructor.
         */
        virtual ~SerialStreamUnitTests() ;

    protected:

        /**
         * @brief Tests constructor overloads and destructors.
         */
        void testSerialStreamConstructors() ;

        /**
         * @brief Tests correct functionality for opening and closing serial streams.
         */
        void testSerialStreamOpenClose() ;

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcdrain().
         */
        void testSerialStreamDrainWriteBuffer() ;

        /**
         * @brief Tests correct functionality for draining the hardware input (read) buffer using tcflush() ;
         */
        void testSerialStreamFlushInputBuffer() ;

        /**
         * @brief Tests correct functionality for draining the hardware write buffer using tcflush() ;
         */
        void testSerialStreamFlushOutputBuffer() ;

        /**
         * @brief Tests correct functionality for draining the hardware read/write buffers using tcflush() ;
         */
        void testSerialStreamFlushIOBuffers() ;

        /**
         * @brief Tests correct functionality for checking if data is available.
         */
        void testSerialStreamIsDataAvailableTest() ;

        /**
         * @brief Tests for correct functionality of the Open() and Close() methods.
         */
        void testSerialStreamIsOpenTest() ;

        /**
         * @brief Tests for correct functionality of the SetBaudRate() and GetBaudRate() methods.
         */
        void testSerialStreamSetGetBaudRate() ;

        /**
         * @brief Tests for correct functionality of the SetCharacterSize() and GetCharactersize() methods.
         */
        void testSerialStreamSetGetCharacterSize() ;

        /**
         * @brief Tests for correct functionality of the SetFlowControl() and GetFlowControl() methods.
         */
        void testSerialStreamSetGetFlowControl() ;

        /**
         * @brief Tests for correct functionality of the SetParity() and GetParity() methods.
         */
        void testSerialStreamSetGetParity() ;

        /**
         * @brief Tests for correct functionality of the SetStopBits() and GetStopBits() methods.
         */
        void testSerialStreamSetGetStopBits() ;

        /**
         * @brief Tests for correct functionality of the SetVMin() and GetVMin() methods.
         */
        void testSerialStreamSetGetVMin() ;

        /**
         * @brief Tests for correct functionality of the SetVTime() and GetVMin() methods.
         */
        void testSerialStreamSetGetVTime() ;

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDTR() methods.
         */
        void testSerialStreamSetGetDTR() ;

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetRTS() methods.
         */
        void testSerialStreamSetGetRTS() ;

        /**
         * @brief Tests for correct functionality of the SetRTS() and GetCTS() methods.
         */
        void testSerialStreamSetRTSGetCTS() ;

        /**
         * @brief Tests for correct functionality of the SetDTR() and GetDSR() methods.
         */
        void testSerialStreamSetDTRGetDSR() ;

        /**
         * @brief Tests for correct functionality of the GetFileDescriptor() method.
         */
        void testSerialStreamGetFileDescriptor() ;

        /**
         * @brief Tests for correct functionality of the GetNumberOfBytesAvailable() method.
         */
        void testSerialStreamGetNumberOfBytesAvailable() ;

#ifdef __linux__
        /**
         * @brief Tests for correct functionality of the GetAvailableSerialPorts() method.
         */
        void testSerialStreamGetAvailableSerialPorts() ;
#endif

        /**
         * @brief Tests for correct functionality of the ReadByte() and WriteByte() methods.
         */
        void testSerialStreamReadByteWriteByte() ;

        /**
         * @brief Tests for correct functionality of the GetLine() and WriteString() methods.
         */
        void testSerialStreamGetLineWriteString() ;

        /**
         * @brief Tests for correct functionality of the Get() and WriteByte() methods.
         */
        void testSerialStreamGetWriteByte() ;

    } ; // class SerialStreamUnitTests

} // namespace LibSerial
