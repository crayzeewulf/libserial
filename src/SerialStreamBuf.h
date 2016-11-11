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

#include <memory>


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
         * @brief Default Constructor.
         */
        explicit SerialStreamBuf();

        /**
         *  @brief Default Destructor.  
         */
        virtual ~SerialStreamBuf();

        /**
         * @brief If IsOpen() != <tt>false</tt>, returns a null
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
        SerialStreamBuf* Open(const std::string& filename,
                              std::ios_base::openmode mode = std::ios_base::in | std::ios_base::out);

        /**
         * @brief If IsOpen() == false, returns a null pointer.
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
         *        <b>Postcondition</b>: IsOpen() == <tt>false<tt>
         *
         * @return Returns <tt>this</tt> on success, a null pointer
         *         otherwise.
         */
        SerialStreamBuf* Close();

        /**
         * @brief Returns true if a previous call to open() succeeded
         *        (returned a non-null value) and there has been no
         *        intervening call to close.
         */
        bool IsOpen();

        /** 
         * @brief This routine is called by open() in order to
         *        initialize some parameters of the serial port and
         *        setting its parameters to default values.
         * @return -1 on failure and some other value on success. 
         */
        int InitializeSerialPort();

        /**
         * @brief Initializes the serial communication parameters to their
         *        default values.
         */
        void SetParametersToDefault();

        /**
         * @brief Sets the baud rate for the serial port to the specified value
         * @param baudRate The baud rate to be set for the serial port.
         */
        void SetBaudRate(const BaudRate& baudRate);

        /**
         * @brief Gets the current baud rate for the serial port.
         * @return Returns the baud rate.
         */
        BaudRate GetBaudRate();

        /**
         * @brief Sets the character size for the serial port.
         * @param characterSize The character size to be set.
         */
        void SetCharacterSize(const CharacterSize& characterSize);

        /**
         * @brief Gets the character size being used for serial communication.
         * @return Returns the current character size. 
         */
        CharacterSize GetCharacterSize();

        /**
         * @brief Sets flow control for the serial port.
         * @param flowControl The flow control type to be set.
         */
        void SetFlowControl(const FlowControl& flowControl);

        /**
         * @brief Get the current flow control setting.
         * @return Returns the flow control type of the serial port.
         */
        FlowControl GetFlowControl();

        /**
         * @brief Sets the parity type for the serial port.
         * @param parityType The parity type to be set.
         */
        void SetParity(const Parity& parityType);

        /**
         * @brief Gets the parity type for the serial port.
         * @return Returns the parity type.
         */
        Parity GetParity();

        /**
         * @brief Sets the number of stop bits to be used with the serial port.
         * @param numberOfStopBits The number of stop bits to set.
         */
        void SetNumberOfStopBits(const StopBits& numberOfStopBits);

        /**
         * @brief Gets the number of stop bits currently being used by the serial
         * @return Returns the number of stop bits.
         */
        StopBits GetNumberOfStopBits();

        /**
         * @brief Sets the minimum number of characters for non-canonical reads.
         * @param vMin the number of minimum characters to be set.
         */
        void SetVMin(const short vmin);

        /**
         * @brief Gets the VMIN value for the device, which represents the
         *        minimum number of characters for non-canonical reads.
         * @return Returns the minimum number of characters for
         *         non-canonical reads.
         */
        short GetVMin();

        /** 
         * @brief Sets character buffer timeout for non-canonical reads in deciseconds.
         * @param vtime The timeout value in deciseconds to be set.
         */
        void SetVTime(const short vtime);

        /** 
         * @brief Gets the current timeout value for non-canonical reads in deciseconds.
         * @return Returns the character buffer timeout for non-canonical reads in deciseconds. 
         */
        short GetVTime();


    protected:
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
         * @brief Writes up to n characters from the character sequence at 
         *        char s to the serial port associated with the buffer. 
         *
         * @return Returns the number of characters that were successfully
         *         written to the serial port. 
         */
        virtual std::streamsize xsputn(const char_type* s, 
                                       std::streamsize  n) override;
           
        /**
         * @brief Reads up to n characters from the serial port and returns
         *        them through the character array located at s.
         * @return Returns the number of characters actually read from the
         *         serial port. 
         */
        virtual std::streamsize xsgetn(char_type*      s, 
                                       std::streamsize n) override;

        /**
         * @brief Writes the specified character to the associated serial port.
         * @param character The character to be written to the serial port.
         * @return Returns the character. 
         */
        virtual int_type overflow(const int_type character) override;

        /**
         * @brief Reads and returns the next character from the associated
         *        serial port if one otherwise returns traits::eof(). This
         *        method is used for buffered I/O while uflow() is called
         *        for unbuffered I/O.
         * @return Returns the next character from the serial port. 
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
         * @param character The character to putback.
         * @return Returns The character iff successful, otherwise eof to signal an error.
         */
        virtual int_type pbackfail(const int_type character = traits_type::eof()) override;

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
    }; // class SerialStreamBuf
} // namespace LibSerial

#endif // #ifndef _SerialStreamBuf_h_
