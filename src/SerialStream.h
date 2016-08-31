/***************************************************************************
 *   @file SerialStream.h                                                  *
 *   @copyright                                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _SerialStream_h_
#define _SerialStream_h_

#include <cassert>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <string>
#include <termios.h>

#include <SerialStreamBuf.h>

extern "C++" 
{
    namespace LibSerial 
    {
        /**
         * @brief A stream class for accessing serial ports on POSIX operating
         *        systems. A lot of the functionality of this class has been obtained
         *        by looking at the code of libserial package by Linas Vepstas,
         *        (linas@linas.org) and the excellent document on serial programming
         *        by Michael R. Sweet. This document can be found at
         *        <ahref="http://www.easysw.com/~mike/serial/serial.html">
         *        http://www.easysw.com/~mike/serial/serial.html</a>.
         *        The libserial package can be found at
         *        <ahref="http://www.linas.org/serial/">
         *        http://www.linas.org/serial/</a>.
         *        This class allows one to set various parameters of a serial port
         *        and then access it like a simple fstream. (In fact, that is exactly
         *        what it does!) It sets the parameters of the serial port by
         *        maintaining a file descriptor for the port and uses the
         *        basic_fstream functions for the IO. We have not implemented any
         *        file locking yet but it will be added soon.
         *
         *        Make sure you read the documentation of the standard fstream
         *        template before using this class because most of the
         *        functionality is inherited from fstream. Also a lot of
         *        information about the various system calls used in the
         *        implementation can also be found in the Single Unix
         *        Specification (Version 2). A copy of this document can be
         *        obtained from <a href="http://www.UNIX-systems.org/">
         *        http://www.UNIX-systems.org/</a>. We will refer to this
         *        document as SUS-2.
         *
         * @author $Author: crayzeewulf $ <A HREF="pagey@gnudom.org">Manish P. Pagey</A>
         * @version $Id: SerialStream.h,v 1.5 2004/05/06 18:32:02 crayzeewulf
         *
         */
        class SerialStream : public std::iostream 
        {
        public:
            /**------------------------------------------------------------
             * Public Static Members
             * ------------------------------------------------------------ 
             */
            
            /**
             * @brief This constructor takes a filename and an openmode to
             *        construct a SerialStream object. This results in a
             *        call to basic_fstream::open(s,mode). This is the only
             *        way to contruct an object of this class. We have to
             *        enforce this instead of providing a default
             *        constructor because we want to get a file descriptor
             *        whenever the basic_fstream::open() function is
             *        called. However, this function is not made virtual in
             *        the STL hence it is probably not very safe to overload
             *        it. We may decide to overload it later but the users
             *        of this class will have to make sure that this class
             *        is not used as an fstream class. The SerialStream will
             *        be in the "open" state (same state as after calling
             *        the Open() method) after calling this constructor.
             *
             *        If the constructor has problems opening the serial port or
             *        getting the file-descriptor for the port, it will set the
             *        failbit for the stream. So, one must make sure that the
             *        stream is in a good state before using it for any further
             *        I/O operations.
             *
             * @param fileName The filename of the serial port. 
             * @param openMode The openmode for the serial port file. 
             *
             */
            explicit SerialStream(const std::string fileName, 
                                  std::ios_base::openmode openMode = std::ios::in|std::ios::out);

            /**
             * @brief Constructor that allows one to create a SerialStream
             *        instance and also initialize the corresponding serial
             *        port with the specified parameters. This was suggested
             *        by Witek Adamus (wit3k). 
             * 
             * @note See https://sourceforge.net/tracker/index.php?func=detail&aid=2137885&group_id=9432&atid=359432
             *
             * @param fileName The file descriptor of the serial stream object.
             * @param baudRate The communications baud rate.
             * @param characterSize The size of the character buffer for storing read/write streams.
             * @param parityType The parity type for the serial stream object.
             * @param numberOfStopBits The number of stop bits.
             * @param flowControlType Flow control for the serial data stream.
             */
            SerialStream(const std::string fileName,
                         const SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::DEFAULT_BAUD,
                         const SerialStreamBuf::CharSizeEnum characterSize = SerialStreamBuf::DEFAULT_CHAR_SIZE,
                         const SerialStreamBuf::ParityEnum parityType = SerialStreamBuf::DEFAULT_PARITY,
                         const short numberOfStopBits = SerialStreamBuf::DEFAULT_NO_OF_STOP_BITS,
                         const SerialStreamBuf::FlowControlEnum flowControlType = SerialStreamBuf::DEFAULT_FLOW_CONTROL);

            /**
             * @brief Default Contructor.  Creates a new SerialStream object but does not open it.
             *        The Open() method will need to be called explicitly on the object to
             *        communicate with the serial port.
             */
            explicit SerialStream();
      
            /**
             * @brief Default Destructor. Closes the stream associated with mFileDescriptor.
             *        Remaining actions are accomplished by the fstream destructor.
             */
            virtual ~SerialStream(); 

            /* -----------------------------------------------------------------
             * Other Public Methods
             * -----------------------------------------------------------------
             */
            /**
             * @brief Opens the serial port associated with the specified filename, and the specified mode, mode.
             * @param fileName The file descriptor of the serial stream object.
             * @param openMode The communication mode status when the serial communication port is opened.
             */
            void Open(const std::string fileName, 
                      std::ios_base::openmode openMode = std::ios_base::in | std::ios_base::out);

            /**
             * @brief Closes the serial port. No communications can occur with the serial port 
             *        after calling this routine.
             */
            void Close();

            /**
             * @brief Determines if the Serial Stream is in an open state.
             * @return Returns true iff the Stream is in an open state.
             */
            bool IsOpen() const;

            /** 
             * @brief Sets the input and output baud ratesfor the Serial Stream object. 
             */
            void SetBaudRate(const SerialStreamBuf::BaudRateEnum baudRate);

            /**
             * @brief Gets the current baud rate being used for serial communication.
             *        This routine queries the serial port for its current settings
             *        and returns the baud rate that is being used by the serial port. 
             *
             * @note This is not a constant function because it checks to see that
             *       it is dealing with a SerialStream with a non-null buffer.
             *       If the buffer is null, it attempts to set the state of the stream accordingly.
             *
             * @return Returns the current baud rate for the serial port.
             */
            SerialStreamBuf::BaudRateEnum BaudRate();

            /**
             * @brief Sets the character size associated with the serial port. 
             * @param characterSize The character size will be set to this value. 
             */
            void SetCharSize(const SerialStreamBuf::CharSizeEnum characterSize);

            /**
             * @brief Gets the character size being used for serial communication.
             * @return Returns the current character size. 
            */
            SerialStreamBuf::CharSizeEnum CharSize();

            /**
             * @brief Sets the number of stop bits used during serial communication.
             *        The only valid values are 1 and 2.
             * @param numberOfStopBits The number of stop bits. (1 or 2). 
             */
            void SetNumOfStopBits(const short numberOfStopBits);

            /**
             * @brief Gets the number of stop bits being used during serial communication.
             * @return Returns the number of stop bits.
             */
            short NumOfStopBits(); 

            /**
             * @brief Sets the parity type for serial communication.
             * @param parityType The parity value. 
             */
            void SetParity(const SerialStreamBuf::ParityEnum parityType);

            /**
             * @brief Get the current parity setting for the serial port. 
             * @return Returns the parity setting for the serial port. 
             */
            SerialStreamBuf::ParityEnum Parity();

            /**
             * @brief Sets the specified flow control type. 
             */
            void SetFlowControl(const SerialStreamBuf::FlowControlEnum flowControlType);

            /**
             * @brief Returns the current flow control setting.
             * @return Returns the current flow control setting.
             */
            SerialStreamBuf::FlowControlEnum FlowControl();

            /**
             * @brief Sets character buffer size.
             * @param vMin The size to set the read/write character buffer.
             * @return Returns 
             */
            short SetVMin(const short vMin);


            /**
             * @brief Gets current size of character buffer.
             *        See <A HREF="http://www.unixwiz.net/techtips/termios-vmin-vtime.html">here</A>
             *        for more documentation about VTIME and VMIN.
             * @return Returns 
             */
            short VMin();

            /**
             * @brief Sets the character buffer timing in 10ths of a second.
             * @param vTime The character buffer timing value to be set.
             * @return Returns 
             */
            short SetVTime(const short vTime);

            /**
             * @brief Get current timing of character buffer in 10th of a second.
             *        See <A HREF="http://www.unixwiz.net/techtips/termios-vmin-vtime.html">here</A>
             *        for more documentation about VTIME and VMIN.
             * @return Returns 
             */
            short VTime();


            /**------------------------------------------------------------
             * Friends
             * ------------------------------------------------------------
             */
        protected:
            /**------------------------------------------------------------
             * Protected Data Members
             * ------------------------------------------------------------
             */
            /**------------------------------------------------------------
             * Protected Methods
             * ------------------------------------------------------------
             */
        private:
            /**------------------------------------------------------------
             * Private Data Members
             * ------------------------------------------------------------
             */
            
            /**
             * The copy constructor and the assignment operator are declared but never defined.
             * This allows the compiler to catch any attempts to copy instances of this class.
             */


            /**
             * @brief
             */ 
            // SerialStream(const SerialStream&);
            // SerialStream& operator=(const SerialStream&);

            /**
             * @brief The SerialStreamBuffer object that will be used by the stream
             *        to communicate with the serial port.
             */
            SerialStreamBuf* mIOBuffer;

            /**----------------------------------------------------------------
             * Private Methods
             * ----------------------------------------------------------------
             */
            
            /**
             * @brief Sets the serial port to ignore the modem status lines. If the
             *        specified boolean parameter is false then the meaning of
             *        this function is reversed i.e. the serial port will start
             *        using the modem status lines.
             *
             *  @param ignore If true then the modem status lines will be
             *  ignored otherwise they will be used during the
             *  communication.
             */
            //void IgnoreModemStatusLines(bool ignore=true);

            /**
             * @brief Enables the serial port receiver. This will allow us to read
             *        data from the serial port.
             * @param enable If true then the received will be enabled. Otherwise it will be disabled.
             */
            //void EnableReceiver(bool enable=true);

        }; // class SerialStream

    } // namespace LibSerial

} // extern "C++"

#endif // #ifndef _SerialStream_h_
