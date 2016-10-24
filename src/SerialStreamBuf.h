/******************************************************************************
 *   @file SerialStreamBuf.h                                                  *
 *   @copyright                                                               *
 *                                                                            *
 *   This program is free software; you can redistribute it and/or modify     *
 *   it under the terms of the GNU General Public License as published by     *
 *   the Free Software Foundation; either version 2 of the License, or        *
 *   (at your option) any later version.                                      *
 *                                                                            *
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU General Public License for more details.                             *
 *                                                                            *
 *   You should have received a copy of the GNU General Public License        *
 *   along with this program; if not, write to the                            *
 *   Free Software Foundation, Inc.,                                          *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.                *
 *****************************************************************************/

#ifndef _SerialStreamBuf_h_
#define _SerialStreamBuf_h_

#include "SerialPortConstants.h"
#include "SerialPort.h"
#include <memory>
#include <streambuf>
#include <string>
#include <limits>

namespace LibSerial 
{
    /**
     * @brief This is the streambuf subclass used by SerialStream. This
     *        subclass takes care of opening the serial port file in the
     *        required modes and providing the corresponding file
     *        descriptor to SerialStream so that various parameters
     *        associated with the serial port can be set. Several
     *        features of this streambuf class resemble those of
     *        std::filebuf, however this class it not made a subclass of
     *        filebuf because we need access to the file descriptor
     *        associated with the serial port and the standard filebuf
     *        does not provide access to it.
     *
     *        At present, this class uses unbuffered I/O and all calls
     *        to setbuf() will be ignored.
     */
    class SerialStreamBuf : public std::streambuf 
    {
    public:

        /**
         * @brief The allowed parity values associated with
         *        serial port communications.
         * @deprecated This enumeration is deprecated and will be
         *             removed in version 0.7.0.
         *             Please use SerialPort::Parity instead.
         */
        enum ParityEnum 
        {
            PARITY_EVEN = SerialPort::PARITY_EVEN,
            PARITY_ODD  = SerialPort::PARITY_ODD,
            PARITY_NONE = SerialPort::PARITY_NONE,
            PARITY_DEFAULT = SerialPort::PARITY_DEFAULT,
            PARITY_INVALID   //!< Invalid parity value.
        };


        /**
         * @brief The flow control setting values for a serial port.
         *
         * @deprecated This enumeration has been deprecated and
         *             will be removed in version 0.7.0. Please use
         *             SerialPort::FlowControl instead.
         */
        enum FlowControlEnum 
        {
            FLOW_CONTROL_HARD    = SerialPort::FLOW_CONTROL_HARD,
            FLOW_CONTROL_SOFT    = SerialPort::FLOW_CONTROL_SOFT,
            FLOW_CONTROL_NONE    = SerialPort::FLOW_CONTROL_NONE,
            FLOW_CONTROL_DEFAULT = SerialPort::FLOW_CONTROL_DEFAULT,
            FLOW_CONTROL_INVALID //!< Invalid flow control setting. 
        };

        /**
             * @brief The default value of the baud rate of the serial port.
             *
             * @deprecated Please use SerialPort::BAUD_DEFAULT instead.
             */
        static constexpr BaudRate DEFAULT_BAUD = BaudRate::BAUD_DEFAULT;

        /** 
         * @brief The default value of the character size used during the
         *        serial communication.
         *
         * @deprecated Please use SerialPort::CHAR_SIZE_DEFAULT instead.
         */
        static constexpr CharSize DEFAULT_CHAR_SIZE = CharSize::CHAR_SIZE_DEFAULT;

        /** 
         * @brief The default number of stop bits used.
         *
         * @deprecated Please use SerialPort::STOP_BITS_DEFAULT instead.
         */
        static constexpr short DEFAULT_NO_OF_STOP_BITS = 1;

        /** 
         * @brief The default parity setting.
         *
         * @deprecated Please use SerialPort::PARITY_DEFAULT instead.
         */
        static constexpr ParityEnum DEFAULT_PARITY = SerialStreamBuf::PARITY_DEFAULT;

        /**
         * @brief The default flow control setting.
         *
         * @deprecated Please use SerialPort::FLOW_CONTROL_DEFAULT instead.
         */ 
        static constexpr FlowControlEnum DEFAULT_FLOW_CONTROL = SerialStreamBuf::FLOW_CONTROL_DEFAULT;

        /**
         * @brief The default character buffer size.
         *
         * @deprecated VMIN and VTIME will not be supported starting
         *             from version 0.7.0. Methods of SerialPort class
         *             provide better mechanisms for implementing read
         *             and write timeouts.
         */
        static constexpr short DEFAULT_VMIN = 1;

        /**
         * @brief The default character buffer timing.
         *
         * @deprecated VMIN and VTIME will not be supported starting
         *             from version 0.7.0. Methods of SerialPort class
         *             provide better mechanisms for implementing read
         *             and write timeouts.
         */
        static constexpr short DEFAULT_VTIME = 0;

        /**
         * @brief Default Constructor.
         */
        SerialStreamBuf();

        /**
         *  @brief Default Destructor.  
         */
        ~SerialStreamBuf();

        /**
         * @brief Returns true if a previous call to open() succeeded
         *        (returned a non-null value) and there has been no
         *        intervening call to close.
         */
        bool is_open() const ;

        /**
         * @brief If is_open() != <tt>false</tt>, returns a null
         *        pointer. Otherwise, initializes the <tt>streambuf</tt>
         *        as required. It then opens a file, if possible, whose
         *        name is given as the string <tt>filename</tt> using the
         *        system call <tt>std::open(filename.c_str(), flags)</tt>.
         *        The value of parameter <tt>flags</tt> is obtained from
         *        the value of the parameter mode. At present, only
         *        <tt>ios_base::in</tt> , <tt>ios_base::out</tt> , and
         *        (<tt>ios_base::in|ios_base::out</tt>) make sense for a
         *        serial port and hence all other settings result in the
         *        call to fail. The value of <tt>flags</tt> is obtained as:
         *        <br>
         *
         *        <tt>flags = u_flags | O_NOCTTY</tt>
         *        <br>
         *
         *        where <tt>u_flags</tt> is obtained from the following
         *        table depending on the value of the parameter mode:
         *
         *        <table align="center">
         *        <tr>
         *        <td> <b><tt>in</tt></b>      </td>
         *        <td> <b><tt>out</tt></b>     </td>
         *        <td> <b><tt>u_flags</tt></b> </td>
         *        </tr>
         *        <tr>
         *        <td> + </td>
         *        <td> </td>
         *        <td> <tt>O_RDONLY</tt> </td>
         *        </tr>
         *        <tr>
         *        <td> </td>
         *        <td> + </td>
         *        <td> <tt>O_WRONLY</tt> </td>
         *        </tr>
         *        <tr>
         *        <td> + </td>
         *        <td> + </td>
         *        <td> <tt>O_RDWR</tt> </td>
         *        </tr>
         *        </table>
         *
         * @return Returns <tt>this</tt> on success, a null pointer
         *         otherwise.
         */
        SerialStreamBuf* open( const std::string filename,
                               std::ios_base::openmode mode =
                               std::ios_base::in | std::ios_base::out ) ;

        /**
         * @brief If is_open() == false, returns a null pointer.
         *        If a put area exists, calls overflow(EOF) to flush
         *        characters. Finally it closes the file by calling 
         *        <tt>std::close(mFileDescriptor)</tt> where
         *        mFileDescriptor is the value returned by the last call
         *        to Open().
         *
         *        For the implementation of the corresponding function in
         *        class filebuf, if the last virtual member function called
         *        on <tt>*this</tt> (between underflow, overflow,
         *        <tt>seekoff</tt>, and <tt>seekpos</tt>) was overflow then
         *        it calls <tt>a_codecvt.unshift</tt> (possible several
         *        times) to determine a termination sequence, inserts those
         *        characters and calls overflow(EOF) again. However,
         *        <b>this is not implemented here yet</b>.
         *
         *        <b>Postcondition</b>: is_open() == <tt>false<tt>
         *
         * @return Returns <tt>this</tt> on success, a null pointer
         *         otherwise.
         */
        SerialStreamBuf* close();

        /**
         * @brief Initializes the serial communication parameters to their
         *        default values.
         */
        int SetParametersToDefault();

        /**
         * @brief Sets the baud rate of the associated serial port unless 
         *        is_open() != true, then returns -1.
         * @param baudRate The baud rate value to be set.
         * @return Returns the baud rate on success and BAUD_INVALID on
         *         failure.
         */
        BaudRate SetBaudRate(const BaudRate baudRate );

        /**
         * @return Returns the current baud rate of the serial port.
         *         If the baud rate is not set to a valid value then it
         *         returns BAUD_INVALID.
         */
        BaudRate GetBaudRate() const;

        /**
         * @brief Sets the character size to be used during serial
         *        communication.
         * @param characterSize The size of the character to be set.
         * @return Returns the character size on success and
         *         CHAR_SIZE_INVALID on failure.
         */
        CharSize SetCharSize(const CharSize charSize);

        /**
         * @brief Gets the character size currently being used during
         *        serial communication.
         * @return Returns the character size currently being used during
         *         serial communication.
         */
        CharSize GetCharSize() const;

        /**
         * @brief Sets the number of stop bits used during serial
         *        communication. The only valid values are 1 and 2.
         * @param numberOfStopBits The number of stop bits. (1 or 2). 
         * @return Returns the number of stop bits or -1 on failure.
         */
        short SetNumOfStopBits(const short numOfStopBits);

        /**
         * @brief Get the number of stop bits being used during serial
         *        communication.
         * @return Returns the number of stop bits.  
         */
        short GetNumOfStopBits() const;

        /**
         * @brief Sets the parity for serial communication.
         * @param parityType The parity type value to be set. 
         * @return Returns the parity type set.
         */
        ParityEnum SetParity(const ParityEnum parityType);

        /**
         * @brief Gets the current parity setting for the serial port.
         * @return Returns the parity setting for the serial port.
         */
        ParityEnum GetParity() const;

        /**
         * @brief Sets the specified flow control.
         * @param flowControlType The flow control type to be set.
         * @return Returns the flow control type set.
         */
        FlowControlEnum SetFlowControl(const FlowControlEnum flowControlType);

        /**
         * @brief Gets the current flow control setting.
         * @return Returns the current flow control setting.
         */
        FlowControlEnum GetFlowControl() const;

        /**
         * @brief Sets the minimum number of characters for non-canonical
         *        reads.
         * @note See VMIN in man termios(3).
         * @param vMin the number of minimum characters to be set.
         * @return Returns the minimum number of charcters set.
         */
        short SetVMin(const short vmin);

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for
         *         non-canonical reads.
         */
        short GetVMin() const;

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in
         *        deciseconds.
         * @param vtime The timeout value (in deciseconds) to be set.
         * @return Returns the character buffer timeout for non-canonical
         *         reads in deciseconds.
         */
        short SetVTime(const short vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in
         *        deciseconds.
         * @return Returns the character buffer timeout for non-canonical
         *         reads in deciseconds. 
         */
        short GetVTime() const;

    protected:
        /**
         * @brief Character used to signal that I/O can start while using
         *        software flow control with the serial port.
         */
        static const char CTRL_Q = 0x11;
  
        /**
         * @brief Character used to signal that I/O should stop while using
         *        software flow control with the serial port.
         */
        static const char CTRL_S = 0x13;

        /**
         * @brief Performs an operation that is defined separately for each
         *        class derived from streambuf. The default behavior is to
         *        do nothing if gptr() is non-null and gptr()!=egptr().
         *        Also, setbuf(0, 0) usually means unbuffered I/O and
         *        setbuf(p, n) means use p[0]...p[n-1] to hold the buffered
         *        characters. In general, this method implements the
         *        subclass's notion of getting memory for the buffered
         *        characters. 
         *
         *        In the case of SerialStreamBuffer, we want to keep using
         *        unbuffered I/O. Hence, using this method has no effect at
         *        present.
         */
        virtual std::streambuf* setbuf(char_type*, 
                                       std::streamsize) override;

        /**
         * @brief Reads upto n characters from the serial port and returns
         *        them through the character array located at s.
         * @return Returns the number of characters actually read from the
         *         serial port. 
         */
        virtual std::streamsize xsgetn( char_type*      s, 
                                        std::streamsize n ) override ;

        /**
         * @brief Writes upto n characters from the character sequence at 
         *        char s to the serial port associated with the buffer. 
         *
         * @return Returns the number of characters that were successfully
         *         written to the serial port. 
         */
        virtual std::streamsize xsputn(const char_type* s, 
                                       std::streamsize  n) override;
           
        /**
         * @brief Checks wether input is available on the port.
         *        If you call \c SerialStream::in_avail, this method will
         *        be called to check for available input.
         *        \code
         *        while(serial_port.rdbuf()->in_avail() > 0)
         *        {
         *            serial_port.get(ch);
         *            ...
         *        }
         *        \endcode
         */
        virtual std::streamsize showmanyc() override;
 
        /**
         * @brief Writes the specified character to the associated serial
         *        port.
         * @param c The character to be written to the serial port.
         * @return Returns the character. 
         */
        virtual int_type overflow(int_type c) override;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for buffered I/O while uflow() is called
         *        for unbuffered I/O.
         * @return The next character from the serial port. 
         */
        virtual int_type underflow() override;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for unbuffered I/O while underflow() is
         *        called for unbuffered I/O.
         * @return Returns the next character from the serial port.  
         */
        virtual int_type uflow() override;

        /**
         * @brief This function is called when a putback of a character
         *        fails. This must be implemented for unbuffered I/O as all
         *        streambuf subclasses are required to provide putback of
         *        at least one character.
         * @return Returns ???
         */
        virtual int_type pbackfail(int_type c = traits_type::eof()) override;

        /**
         * @brief Copying and moving of instances of this class are prohibited.
         *        This allows the compiler to catch attempts to copy instances
         *        of this class.
         */
        SerialStreamBuf(const SerialStreamBuf&) = delete;
        SerialStreamBuf(SerialStreamBuf&&) = delete;

        SerialStreamBuf& operator=(const SerialStreamBuf&) = delete;
        SerialStreamBuf& operator=(SerialStreamBuf&&) = delete;

    private:
        class Implementation;
        std::unique_ptr<Implementation> mImpl;
    } ; // class SerialStreamBuf
} // namespace LibSerial

#endif // #ifndef _SerialStreamBuf_h_
