/******************************************************************************
 *   @file SerialPortConstants.h                                              *
 *   @copyright (C) 2015 Manish Pagey                                         *
 *   crayzeewulf@users.sourceforge.net                                        *
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

#ifndef SERIALPORTCONSTANTS_H
#define SERIALPORTCONSTANTS_H

#include <cerrno>
#include <iostream>
#include <limits>
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
    const int MICROSECONDS_PER_MS  =    1000;
    const int MILLISECONDS_PER_SEC =    1000;
    const int MICROSECONDS_PER_SEC = 1000000;
        
    /**
     * @brief The default character buffer size.
     */
    static constexpr short VMIN_DEFAULT = 1;

    /**
     * @brief The default character buffer timing.
     */
    static constexpr short VTIME_DEFAULT = 0;

    /**
     * @brief Character used to signal that I/O can start while using
     *        software flow control with the serial port.
     */
    static constexpr char CTRL_Q = 0x11;

    /**
     * @brief Character used to signal that I/O should stop while using
     *        software flow control with the serial port.
     */
    static constexpr char CTRL_S = 0x13;

    /**
     * @brief Type used to receive and return raw data to/from methods.
     */
    typedef std::vector<unsigned char> DataBuffer;


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
        NotOpen(const std::string& whatArg)
            : logic_error(whatArg)
        {
        }
    };

    /**
     * @brief Exception error thrown when the serial port is already open.
     */
    class AlreadyOpen : public std::logic_error
    {
    public:
        /**
         * @brief Exception error thrown when the serial port is already open.
         */
        AlreadyOpen(const std::string& whatArg)
            : logic_error(whatArg)
        {
        }
    };

    /**
     * @brief Exception error thrown when the serial port could not be opened.
     */
    class OpenFailed : public std::runtime_error
    {
    public:
        /**
         * @brief Exception error thrown when the serial port could not be opened.
         */
        OpenFailed(const std::string& whatArg)
            : runtime_error(whatArg)
        {
        }
    };

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
        ReadTimeout(const std::string& whatArg)
            : runtime_error(whatArg)
        {
        }
    };

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
#endif /* __MAX_BAUD */
#endif /* __linux__ */
        BAUD_DEFAULT = BAUD_115200,
        BAUD_INVALID = std::numeric_limits<speed_t>::max()
    };

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
    };

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
    };

    /**
     * @brief The allowed parity types.
     */
    enum class Parity : tcflag_t
    {
        PARITY_EVEN,                    // Even parity.
        PARITY_ODD,                     // Odd parity.
        PARITY_NONE,                    // No parity i.e. parity checking disabled.
        PARITY_DEFAULT = PARITY_NONE,   // No parity i.e. parity checking disabled.
        PARITY_INVALID = std::numeric_limits<tcflag_t>::max() //!< Invalid parity value.
    };

    /**
     * @brief The allowed number of stop bits.
     */
    enum class StopBits : tcflag_t
    {
        STOP_BITS_1,                     // 1 stop bit.
        STOP_BITS_2,                     // 2 stop bits.
        STOP_BITS_DEFAULT = STOP_BITS_1, // 1 stop bit.
        STOP_BITS_INVALID = std::numeric_limits<tcflag_t>::max()
    };

} // namespace LibSerial

#endif // #ifndef SERIALPORTCONSTANTS_H */