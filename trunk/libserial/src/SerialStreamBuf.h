/*
 * Time-stamp: <04/05/05 16:21:47 pagey>
 *
 * $Id: SerialStreamBuf.h,v 1.5 2005-08-31 12:48:24 wedesoft Exp $
 *
 *
 */
#ifndef _SerialStreamBuf_h_
#define _SerialStreamBuf_h_

#ifndef _termios_h_INCLUDED_
#    include <termios.h>
#    define _termios_h_INCLUDED_
#endif

#ifndef _unistd_h_INCLUDED_
#    include <unistd.h>
#    define _unistd_h_INCLUDED_
#endif

#ifndef _std_iosfwd_INCLUDED_
#    include <iosfwd>
#    define _std_iosfwd_INCLUDED_
#endif

#ifndef _std_streambuf_INCLUDED_
#    include <streambuf>
#    define _std_streambuf_INCLUDED_
#endif

#ifndef _std_string_INCLUDED_
#    include <string>
#    define _std_string_INCLUDED_
#endif

extern "C++" {
    namespace LibSerial {
        /** This is the streambuf subclass used by SerialStream. This
         *  subclass takes care of opening the serial port file in the
         *  required modes and providing the corresponding file
         *  descriptor to SerialStream so that various parameters
         *  associated with the serial port can be set. Several
         *  features of this streambuf class resemble those of
         *  std::filebuf, however this class it not made a subclass of
         *  filebuf because we need access to the file descriptor
         *  associated with the serial port and the standard filebuf
         *  does not provide access to it.
         *
         *  At present, this class uses unbuffered I/O and all calls to
         *  setbuf() will be ignored.
         *
         * @author $Author: wedesoft $ <A HREF="pagey@gnudom.org">Manish P. Pagey</A>
         * @version $Id: SerialStreamBuf.h,v 1.5 2005-08-31 12:48:24 wedesoft Exp $
         * */
        class SerialStreamBuf : public std::streambuf {
        public:
            /** @name Typedefs
             */
            //@{

            //@}

            /** @name Enumerations
             */
            //@{
            /** The baud rates currently supported by the SUS-2 general
                terminal interface specification. Note that B0 is not
                supported because it is not really a baud rate (it causes
                the modem to hang up i.e. drop DTR). Use the close() method
                instead.

            */
            enum BaudRateEnum {
                BAUD_50    = B50,         //!< 50 baud. 
                BAUD_75    = B75,         //!< 75 baud.
                BAUD_110   = B110,        //!< 110 baud.
                BAUD_134   = B134,        //!< 134.5 baud. Yes 134.5. I did not mistype that. 
                BAUD_150   = B150,        //!< 150 baud.
                BAUD_200   = B200,        //!< 200 baud.
                BAUD_300   = B300,        //!< 300 baud.
                BAUD_600   = B600,        //!< 600 baud. 
                BAUD_1200  = B1200,       //!< 1200 baud. 
                BAUD_1800  = B1800,       //!< 1800 baud.
                BAUD_2400  = B2400,       //!< 2400 baud.
                BAUD_4800  = B4800,       //!< 4800 baud.
                BAUD_9600  = B9600,       //!< 9600 baud.
                BAUD_19200 = B19200,      //!< 19200 baud.
                BAUD_38400 = B38400,      //!< 38400 baud.
                BAUD_57600 = B57600,      //!< 57600 baud.
                BAUD_115200 = B115200,    //!< 115200 baud.
                BAUD_INVALID              //!< Invalid baud rate.
            } ;

            /** The allowed values of character sizes that can be used
                during the serial communication.

            */
            enum CharSizeEnum {
                CHAR_SIZE_5 = CS5, //!< 5 bit characters. 
                CHAR_SIZE_6 = CS6, //!< 6 bit characters. 
                CHAR_SIZE_7 = CS7, //!< 7 bit characters. 
                CHAR_SIZE_8 = CS8, //!< 8 bit characters. 
                CHAR_SIZE_INVALID  //!< Invalid character size.  
            } ;

            /** The allowed values of the parity associated with the serial
                port communications.
	  
            */
            enum ParityEnum {
                PARITY_EVEN,     //!< Even parity.  
                PARITY_ODD,      //!< Odd parity.
                PARITY_NONE,     //!< No parity i.e. parity checking disabled.
                PARITY_INVALID   //!< Invalid parity value.
            } ;      

            /** The values of the flow control settings for a serial
                port.

            */
            enum FlowControlEnum {
                FLOW_CONTROL_HARD,   //!< Hardware flow control.
                FLOW_CONTROL_SOFT,   //!< Software flow control. 
                FLOW_CONTROL_NONE,   //!< No flow control.
                FLOW_CONTROL_INVALID //!< Invalid flow control setting. 
            } ;
            //@}

            /* ------------------------------------------------------------
             * Public Static Members
             * ------------------------------------------------------------
             */
            /** @name Public static members. 

            */
            //@{
            /** The default value of the baud rate of the serial port. 

            */
            static const BaudRateEnum DEFAULT_BAUD ;

            /** The default value of the character size used during the
                serial communication.

            */
            static const CharSizeEnum DEFAULT_CHAR_SIZE ;

            /** The default number of stop bits used.  

            */
            static const short DEFAULT_NO_OF_STOP_BITS ;

            /** The default parity setting. 
	  
            */
            static const ParityEnum DEFAULT_PARITY ;
      
            /** The default flow control setting.

            */
            static const FlowControlEnum DEFAULT_FLOW_CONTROL ;
            //@}


            /** @name Exceptions
             */
            //@{

            //@}

            /** @name Constructors and Destructor
             */
            //@{
            /** The default constructor.

            */
            SerialStreamBuf() ;

            /** The destructor.  

            */
            ~SerialStreamBuf() ;
            //@}

            /** @name Other Public Methods
             */
            //@{
            /** Returns true if a previos call to open() succeeded (returned
                a non-null value) and there has been no intervening call to
                close.

            */
            bool is_open() const ;

            /** If is_open() != <tt>false</tt>, returns a null
                pointer. Otherwise, initializes the <tt>streambuf</tt> as
                required. It then opens a file, if possible, whose name is
                given as the string <tt>filename</tt> using the system call
                <tt>std::open(filename.data(), flags)</tt>. The value of
                parameter <tt>flags</tt> is obtained from the value of the
                parameter mode. At present, only <tt>ios_base::in</tt>,
                <tt>ios_base::out</tt>, and
                (<tt>ios_base::in|ios_base::out</tt>) make sense for a
                serial port and hence all other settings result in the call
                to fail. The value of <tt>flags</tt> is obtained as:
                <br>
	  
                <tt>flags = u_flags | O_NOCTTY</tt>
                <br>

                where <tt>u_flags</tt> is obtained from the following table
                depending on the value of the parameter mode:

                <table align="center">
                <tr>
                <td> <b><tt>in</tt></b>      </td>
                <td> <b><tt>out</tt></b>     </td>
                <td> <b><tt>u_flags</tt></b> </td>
                </tr>
                <tr>
                <td> + </td>
                <td> </td>
                <td> <tt>O_RDONLY</tt> </td>
                </tr>
                <tr>
                <td> </td>
                <td> + </td>
                <td> <tt>O_WRONLY</tt> </td>
                </tr>
                <tr>
                <td> + </td>
                <td> + </td>
                <td> <tt>O_RDWR</tt> </td>
                </tr>
                </table>

                @return If the <tt>open</tt>() system call succeeds the
                method returns <tt>this</tt>. If the call fails, then it
                returns a null pointer.

            */
            SerialStreamBuf* open( const std::string filename, 
                                   std::ios_base::openmode mode = 
                                   std::ios_base::in | std::ios_base::out ) ;

            /** If is_open() == false, returns a null pointer. If a put area
                exists, calls overflow(EOF) to flush characters. Finally it
                closes the file by calling
                <tt>std::close(mFileDescriptor)</tt> where mFileDescriptor
                is the value returned by the last call to open().

                For the implementation of the corresponding function in
                class filebuf, if the last virtual member function called on
                <tt>*this</tt> (between underflow, overflow,
                <tt>seekoff</tt>, and <tt>seekpos</tt>) was overflow then it
                calls <tt>a_codecvt.unshift</tt> (possible several times) to
                determine a termination sequence, inserts those characters
                and calls overflow(EOF) again. However, <b>this is not
                implemented here yet</b>.

                <b>Postcondition</b>: is_open() == <tt>false<tt>

                @return <tt>this</tt> on success, a null pointer otherwise.

            */
            SerialStreamBuf* close() ;

            /** Initialize the serial communication parameters to their
                default values.

            */
            int SetParametersToDefault() ;

            /** If is_open() != true, return -1. Otherwise, set the baud
                rate of the associated serial port. Return the baud rate 
                on success and BAUD_INVALID on failure. 

            */
            const BaudRateEnum SetBaudRate(const BaudRateEnum baud_rate) ;

            /** Return the current baud rate of the serial port. If the baud
                rate is not set to a valid value then it returns
                BAUD_INVALID.

            */
            const BaudRateEnum BaudRate() const ;

            /** Set the character size to be used during serial
                communication. It returns the character size on success and
                CHAR_SIZE_INVALID on failure.

            */
            const CharSizeEnum SetCharSize(const CharSizeEnum char_size) ;

            /** Return the character size currently being used for serial
                communication.

            */
            const CharSizeEnum CharSize() const ;

            /** Set the number of stop bits used during serial
                communication. The only valid values are 1 and 2.

                @param stop_bits The number of stop bits. (1 or 2). 
                @return The number of stop bits or -1 on failure. 
	  
            */
            short SetNumOfStopBits(short stop_bits) ;

            /** Get the number of stop bits being used during serial
                communication.
	  
                @return The number of stop bits.  
            */
            short NumOfStopBits() const ; 

            /** Set the parity for serial communication.
	  
            @param parity The parity value. 
	  
            */
            const ParityEnum SetParity(const ParityEnum parity) ;

            /** Get the current parity setting for the serial port. 
	  
            @return The parity setting for the serial port. 
	  
            */
            const ParityEnum Parity() const ;

            /** Use the specified flow control. 

            */
            const FlowControlEnum SetFlowControl(const FlowControlEnum flow_c) ;

            /** Return the current flow control setting. 

            */
            const FlowControlEnum FlowControl() const ;

            /** Set timeout for reading from port.
                INT_MAX means no timeout. */
            int SetTimeout( int milliseconds ) ;

            /// Return current timeout setting.
            const int Timeout() ;

            //@}

            /** @name Operators
             */
            //@{

            //@}

            /* ------------------------------------------------------------
             * Friends
             * ------------------------------------------------------------
             */
        protected:
            /* ------------------------------------------------------------
             * Protected Data Members
             * ------------------------------------------------------------
             */
            /** Character used to signal that I/O can start while using
                software flow control with the serial port.

            */
            static const char CTRL_Q = 0x11 ;
      
            /** Character used to signal that I/O should stop while using
                software flow control with the serial port.

            */
            static const char CTRL_S = 0x13 ;
            /* ------------------------------------------------------------
             * Protected Methods
             * ------------------------------------------------------------
             */
            /** Performs an operation that is defined separately for each
                class derived from streambuf. The default behavior is to do
                nothing if gptr() is non-null and gptr()!=egptr(). Also,
                setbuf(0, 0) usually means unbuffered I/O and setbuf(p, n)
                means use p[0]...p[n-1] to hold the buffered characters. In
                general, this method implements the subclass's notion of
                getting memory for the buffered characters. 
	  
                In the case of SerialStreamBuf, we want to keep using
                unbuffered I/O. Hence, using this method has no effect at
                present.

            */
            virtual std::streambuf* setbuf( char_type*, 
                                            std::streamsize ) ;

            /** Reads upto n characters from the serial port and returns
                them through the character array located at s.

                @return The number of characters actually read from the
                serial port. 
            */
            virtual std::streamsize xsgetn( char_type*      s, 
                                            std::streamsize n ) ;

	    /** Check, wether input is available on the port.
		If you call \c SerialStream::in_avail, this method will be
		called to check for available input.
		\code
		while( serial_port.rdbuf()->in_avail() > 0  ) {
  		  serial_port.get(ch);
		  ...
		}
		\endcode */
	    virtual std::streamsize showmanyc();

            /** Reads and returns the next character from the associated
                serial port if one otherwise returns traits::eof(). This
                method is used for buffered I/O while uflow() is called for
                unbuffered I/O.

                @return The next character from the serial port. 
            */
            virtual int_type underflow() ;

            /** Reads and returns the next character from the associated
                serial port if one otherwise returns traits::eof(). This
                method is used for unbuffered I/O while underflow() is
                called for unbuffered I/O.

                @return The next character from the serial port.  

            */
            virtual int_type   uflow() ;

            /** This function is called when a putback of a character
                fails. This must be implemented for unbuffered I/O as all
                streambuf subclasses are required to provide putback of at
                lease on character.

            */
            virtual int_type pbackfail(int_type c = traits_type::eof()) ;

            /** Writes upto n characters from the character sequence at s to
                the serial port associated with the buffer. 

                @return The number of characters that were successfully
                written to the serial port. 
            */
            virtual std::streamsize xsputn( const char_type* s, 
                                            std::streamsize  n ) ;

            /** Writes the specified character to the associated
                serial port. 

                @return The character c. 
            */
            virtual int_type overflow(int_type c) ;

        private:
            /* ------------------------------------------------------------
             * Private Data Members
             * ------------------------------------------------------------
             */
            /** We use unbuffered I/O for the serial port. However, we need
                to provide the putback of atleast one character. This
                character contains the putback character.

            */
            char mPutbackChar ;

            /** True if a putback value is available in mPutbackChar. 

            */
            bool mPutbackAvailable ;
      
            /** The file descriptor associated with the serial port. 

            */
            int mFileDescriptor ;

            /// Value for timeout.
            timeval mTimeval;

            /// Boolean, wether timeout is enabled or not.
            bool mTimeout;
            /* ------------------------------------------------------------
             * Private Methods
             * ------------------------------------------------------------
             */
            /** This routine is called by open() in order to initialize some
                parameters of the serial port and setting its parameters to
                default values.

                @return -1 on failure and some other value on success. 
            */
            int InitializeSerialPort() ;
        } ; // class SerialStreamBuf

        inline 
        SerialStreamBuf::SerialStreamBuf() :
            mPutbackChar(0),
            mPutbackAvailable(false),
            mFileDescriptor(-1),
            mTimeout(false)
        {
            setbuf(0, 0) ;
            return ;
        }

        inline 
        SerialStreamBuf::~SerialStreamBuf() 
        {
            if( this->is_open() ) {
                this->close() ;
            }
            return ;
        }

        inline
        bool
        SerialStreamBuf::is_open() const 
        {
            return (-1 != mFileDescriptor) ;
        }
    
        inline
        std::streambuf* 
        SerialStreamBuf::setbuf(char_type *, std::streamsize) 
        {
            return std::streambuf::setbuf(0, 0) ;
        }

        inline
        SerialStreamBuf*
        SerialStreamBuf::close() 
        {
            //
            // Return a null pointer if the serial port is not currently open. 
            //
            if( this->is_open() == false ) {
                return 0 ;
            }
            //
            // Otherwise, close the serial port and set the file descriptor
            // to an invalid value.
            //
            if( -1 == ::close(mFileDescriptor) ) {
                //
                // If the close failed then return a null pointer. 
                //
                return 0 ;
            } else {
                //
                // Set the file descriptor to an invalid value, -1. 
                //
                mFileDescriptor = -1 ;
                //
                // On success, return "this" as required by the C++ standard.
                //
                return this ;
            }
        }
    
        inline
        std::streambuf::int_type
        SerialStreamBuf::uflow() 
        {
            int_type next_ch = underflow() ;
            mPutbackAvailable = false ;
            return next_ch ;
        }

    } ; // namespace LibSerial
} // extern "C++"
#endif // #ifndef _SerialStreamBuf_h_
