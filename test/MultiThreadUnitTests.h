/******************************************************************************
 * @file SerialPortTests.h                                                    *
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

#include "libserial/SerialPort.h"
#include "libserial/SerialPortConstants.h"
#include "UnitTests.h"

#include <gtest/gtest.h>
#include <mutex>

/**
 * @namespace Libserial
 */
namespace LibSerial
{
    class MultiThreadUnitTests : public UnitTests
    {
    public:

        /**
         * @brief Default Constructor.
         */
        explicit MultiThreadUnitTests() ;

        /**
         * @brief Default Destructor.
         */
        virtual ~MultiThreadUnitTests() ;

    protected:

        /**
         * @brief Tests for correct functionality of a multi-threaded serial stream application.
         */
        void serialStream1ThreadLoop() ;

        /**
         * @brief Tests for correct functionality of a multi-threaded serial port application.
         */
        void serialStream2ThreadLoop() ;

        /**
         * @brief Main loop for the multi-threaded serial stream unit test.
         */
        void serialPort1ThreadLoop() ;

        /**
         * @brief Main loop for the multi-threaded serial port unit test.
         */
        void serialPort2ThreadLoop() ;

        /**
         * @brief Entry point for the multi-thread serial stream unit test.
         */
        void testMultiThreadSerialStreamReadWrite() ;

        /**
         * @brief Entry point for the multi-thread serial port unit test.
         */
        void testMultiThreadSerialPortReadWrite() ;

        /**
         * @param C++11 thread std::mutex for locking parameters in the threaded unit tests.
         */
        std::mutex mutex {} ;

    } ; // class MultiThreadUnitTests

} // namespace LibSerial
