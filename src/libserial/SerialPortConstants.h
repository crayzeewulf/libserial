/******************************************************************************
 * @file SerialPortConstants.h                                                *
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

#include <limits>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <vector>

namespace LibSerial
{
    /**
     * @brief Error messages utilized when throwing exceptions.
     */
    const std::string ERR_MSG_INVALID_BAUD_RATE      = "Invalid baud rate.";
    const std::string ERR_MSG_INVALID_CHARACTER_SIZE = "Invalid character size.";
    const std::string ERR_MSG_INVALID_FLOW_CONTROL   = "Invalid flow control.";
    const std::string ERR_MSG_INVALID_PARITY         = "Invalid parity setting.";
    const std::string ERR_MSG_INVALID_STOP_BITS      = "Invalid number of stop bits.";
    const std::string ERR_MSG_READ_TIMEOUT           = "Read timeout";
    const std::string ERR_MSG_PORT_ALREADY_OPEN      = "Serial port already open.";
    const std::string ERR_MSG_PORT_NOT_OPEN          = "Serial port not open.";

    /**
     * @brief Time conversion constants.
     */
    constexpr int MICROSECONDS_PER_MS  =    1000 ;
    constexpr int MILLISECONDS_PER_SEC =    1000 ;
    constexpr int MICROSECONDS_PER_SEC = 1000000 ;

    /**
     * @brief Bits to bytes conversion constant.
     */
    constexpr int BITS_PER_BYTE = 8 ;

    /**
     * @brief The default character buffer size.
     */
    constexpr short VMIN_DEFAULT = 1 ;

    /**
     * @brief The default character buffer timing.
     */
    constexpr short VTIME_DEFAULT = 0 ;

    /**
     * @brief Character used to signal that I/O can start while using
     *        software flow control with the serial port.
     */
    constexpr char CTRL_Q = 0x11 ;

    /**
     * @brief Character used to signal that I/O should stop while using
     *        software flow control with the serial port.
     */
    constexpr char CTRL_S = 0x13 ;

    /**
     * @brief Type used to receive and return raw data to/from methods.
     */
    using DataBuffer =  std::vector<uint8_t> ;


    /**
     * @note - For reference, below is a list of std::exception types:
     *         logic_error
     *         invalid_argument
     *         domain_error
     *         length_error
     *         out_of_range
     *         future_error
     *         runtime_error
     *         range_error
     *         overflow_error
     *         underflow_error
     *         regex_error
     *         system_error
     *         ios_base::failure
     *         bad_typeid
     *         bad_cast
     *         bad_weak_ptr
     *         bad_function_call
     *         bad_alloc
     *         bad_array_new_length
     *         bad_exception
     */

    /**
     * @brief Exception error thrown when the serial port is not open.
     */
    class NotOpen : public std::logic_error
    {
    public:
        /**
         * @brief Exception error thrown when the serial port is not open.
         */
        explicit NotOpen(const std::string& whatArg [[maybe_unused]])
            : logic_error(whatArg)
        {
        }
    } ;

    /**
     * @brief Exception error thrown when the serial port is already open.
     */
    class AlreadyOpen : public std::logic_error
    {
    public:
        /**
         * @brief Exception error thrown when the serial port is already open.
         */
        explicit AlreadyOpen(const std::string& whatArg [[maybe_unused]])
            : logic_error(whatArg)
        {
        }
    } ;

    /**
     * @brief Exception error thrown when the serial port could not be opened.
     */
    class OpenFailed : public std::runtime_error
    {
    public:
        /**
         * @brief Exception error thrown when the serial port could not be opened.
         */
        explicit OpenFailed(const std::string& whatArg [[maybe_unused]])
            : runtime_error(whatArg)
        {
        }
    } ;

    /**
     * @brief Exception error thrown when data could not be read from the
     *        serial port before the timeout had been exceeded.
     */
    class ReadTimeout : public std::runtime_error
    {
    public:
        /**
         * @brief Exception error thrown when data could not be read from the
         *        serial port before the timeout had been exceeded.
         */
        explicit ReadTimeout(const std::string& whatArg [[maybe_unused]])
            : runtime_error(whatArg)
        {
        }
    } ;

    /**
     * @brief The baud rates currently supported by the Single Unix
     *        Specification V3 general terminal interface specification.
     *        Note that B0 is not supported because it not actually a baud
     *        rate, is used to terminate the connection, (i.e. drop DTR).
     *        The Close() method should be used instead.
     */
    enum class BaudRate : speed_t
    {
        BAUD_50      = B50,
        BAUD_75      = B75,
        BAUD_110     = B110,
        BAUD_134     = B134,
        BAUD_150     = B150,
        BAUD_200     = B200,
        BAUD_300     = B300,
        BAUD_600     = B600,
        BAUD_1200    = B1200,
        BAUD_1800    = B1800,
        BAUD_2400    = B2400,
        BAUD_4800    = B4800,
        BAUD_9600    = B9600,
        BAUD_19200   = B19200,
        BAUD_38400   = B38400,
        BAUD_57600   = B57600,
        BAUD_115200  = B115200,
        BAUD_230400  = B230400,

// @note: >B230400 are defined in Linux but not other POSIX systems, (e.g. Mac OS X).
#ifdef __linux__
        BAUD_460800  = B460800,
        BAUD_500000  = B500000,
        BAUD_576000  = B576000,
        BAUD_921600  = B921600,
        BAUD_1000000 = B1000000,
        BAUD_1152000 = B1152000,
        BAUD_1500000 = B1500000,
#if __MAX_BAUD > B2000000
        BAUD_2000000 = B2000000,
        BAUD_2500000 = B2500000,
        BAUD_3000000 = B3000000,
        BAUD_3500000 = B3500000,
        BAUD_4000000 = B4000000,
#endif // __MAX_BAUD
#endif // __linux__
        BAUD_DEFAULT = BAUD_115200,
        BAUD_INVALID = std::numeric_limits<speed_t>::max()
    } ;

    /**
     * @brief The allowed character sizes.
     */
    enum class CharacterSize : tcflag_t
    {
        CHAR_SIZE_5       = CS5, // !< 5 bit characters.
        CHAR_SIZE_6       = CS6, // !< 6 bit characters.
        CHAR_SIZE_7       = CS7, // !< 7 bit characters.
        CHAR_SIZE_8       = CS8, // !< 8 bit characters.
        CHAR_SIZE_DEFAULT = CS8, // !< 8 bit characters.
        CHAR_SIZE_INVALID = std::numeric_limits<tcflag_t>::max()
    } ;

    /**
     * @brief The allowed flow control types.
     */
    enum class FlowControl : tcflag_t
    {
        FLOW_CONTROL_HARDWARE,
        FLOW_CONTROL_SOFTWARE,
        FLOW_CONTROL_NONE,
        FLOW_CONTROL_DEFAULT = FLOW_CONTROL_NONE,
        FLOW_CONTROL_INVALID = std::numeric_limits<tcflag_t>::max()
    } ;

    /**
     * @brief The allowed parity types.
     */
    enum class Parity : tcflag_t
    {
        PARITY_EVEN,                                          // !< Even parity.
        PARITY_ODD,                                           // !< Odd parity.
        PARITY_NONE,                                          // !< No parity i.e. parity checking disabled.
        PARITY_DEFAULT = PARITY_NONE,                         // !< No parity i.e. parity checking disabled.
        PARITY_INVALID = std::numeric_limits<tcflag_t>::max() // !< Invalid parity value.
    } ;

    /**
     * @brief The allowed number of stop bits.
     */
    enum class StopBits : tcflag_t
    {
        STOP_BITS_1,                     // !< 1 stop bit.
        STOP_BITS_2,                     // !< 2 stop bits.
        STOP_BITS_DEFAULT = STOP_BITS_1, // !< 1 stop bit.
        STOP_BITS_INVALID = std::numeric_limits<tcflag_t>::max()
    } ;

} // namespace LibSerial
