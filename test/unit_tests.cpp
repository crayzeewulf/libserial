/******************************************************************************
 * @file unit_tests.cpp                                                       *
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

#include <libserial/SerialStream.h>
#include <libserial/SerialStreamBuf.h>

#define BOOST_TEST_MODULE libserial
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( SerialStreamBuf_Constructor_Test ) // NOLINT (cert-err58-cpp)
{
    // 
    // SerialStreamBuf should not be open on default construction
    //
    using LibSerial::SerialStreamBuf ;
    SerialStreamBuf buf ;
    BOOST_CHECK( false == buf.IsOpen() ) ; // NOLINT (cppcoreguidelines-pro-type-vararg)
}

BOOST_AUTO_TEST_CASE( SerialStream_Constructor_Test ) // NOLINT (cert-err58-cpp)
{
    using LibSerial::SerialStream ;
    //
    // SerialStream should not be open on default construction
    //
    {
        SerialStream serial_stream;
        BOOST_CHECK( false == serial_stream.IsOpen() ) ; // NOLINT (cppcoreguidelines-pro-type-vararg)
    }
    //
    // Attempting to open a non-existent serial port should leave the 
    // serial stream in non-good state.
    //
    {
        SerialStream serial_stream ;
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        BOOST_CHECK_THROW(
            serial_stream.Open("/dev/some_non_existent_device_hope_it_does_not_exist"),
            LibSerial::OpenFailed
        ) ;
        // NOLINTNEXTLINE (cppcoreguidelines-pro-type-vararg)
        BOOST_CHECK(not serial_stream.good()) ;
    }
}

