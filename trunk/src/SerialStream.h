/*
 * Time-stamp: <2008-10-30 16:30:11 pagey>
 *
 * $Id: SerialStream.h,v 1.10 2005-10-01 21:24:21 crayzeewulf Exp $
 *
 *
 */
#ifndef _SerialStream_h_
#define _SerialStream_h_

#include <string>
#include <fstream>
#include <SerialStreamBuf.h>

extern "C++" {
    namespace LibSerial {
        /** A stream class for accessing serial ports on POSIX
            operating systems. A lot of the functionality of this
            class has been obtained by looking at the code of
            libserial package by Linas Vepstas (linas@linas.org) and
            the excellent document on serial programming by Michael
            R. Sweet. This document can be found at <a
            href="http://www.easysw.com/~mike/serial/serial.html">
            http://www.easysw.com/~mike/serial/serial.html</a>. The
            libserial package can be found at <a
            href="http://www.linas.org/serial/">
            http://www.linas.org/serial/</a>. This class allows one to
            set various parameters of a serial port and then access it
            like a simple fstream. In fact, that is exactly what it
            does. It sets the parameters of the serial port by
            maintaining a file descriptor for the port and uses the
            basic_fstream functions for the IO. We have not
            implemented any file locking yet but it will be added
            soon.

            Make sure you read the documentation of the standard fstream
            template before using this class because most of the
            functionality is inherited from fstream. Also a lot of
            information about the various system calls used in the
            implementation can also be found in the Single Unix
            Specification (Version 2). A copy of this document can be
            obtained from <a href="http://www.UNIX-systems.org/">
            http://www.UNIX-systems.org/</a>. We will refer to this
            document as SUS-2.
	
            @author $Author: crayzeewulf $ <A HREF="pagey@gnudom.org">Manish P. Pagey</A>
            @version $Id: SerialStream.h,v 1.5 2004/05/06 18:32:02 crayzeewulf
     
        */
        class SerialStream : public std::iostream {
        public:
            /* ------------------------------------------------------------
             * Public Static Members
             * ------------------------------------------------------------ */

            /** @name Constructors and Destructor
             */
            //@{

            /** This constructor takes a filename and an openmode to
                construct a SerialStream object. This results in a
                call to basic_fstream::open(s,mode). This is the only
                way to contruct an object of this class. We have to
                enforce this instead of providing a default
                constructor because we want to get a file descriptor
                whenever the basic_fstream::open() function is
                called. However, this function is not made virtual in
                the STL hence it is probably not very safe to overload
                it. We may decide to overload it later but the users
                of this class will have to make sure that this class
                is not used as an fstream class. The SerialStream will
                be in the "open" state (same state as after calling
                the Open() method) after calling this constructor.

                If the constructor has problems opening the serial port or
                getting the file-descriptor for the port, it will set the
                failbit for the stream. So, one must make sure that the
                stream is in a good state before using it for any further
                I/O operations.

                @param fileName The filename of the serial port. 
                @param openMode The openmode for the serial port file. 

            */
            explicit SerialStream( const std::string fileName, 
                                   std::ios_base::openmode openMode =
                                   std::ios::in|std::ios::out) ;

            /**
             * Constructor that allows one to create a SerialStream
             * instance and also initialize the corresponding serial
             * port with the specified parameters. This was suggested
             * by Witek Adamus (wit3k). 
             * 
             * See https://sourceforge.net/tracker/index.php?func=detail&aid=2137885&group_id=9432&atid=359432
             *
             * :TODO: Add documentation for all parameters here.
             */
            SerialStream( const std::string fileName,
                          const SerialStreamBuf::BaudRateEnum baudRate = SerialStreamBuf::DEFAULT_BAUD,
                          const SerialStreamBuf::CharSizeEnum charSize = SerialStreamBuf::DEFAULT_CHAR_SIZE,
                          const SerialStreamBuf::ParityEnum parityType = SerialStreamBuf::DEFAULT_PARITY,
                          const short numOfStopBits = SerialStreamBuf::DEFAULT_NO_OF_STOP_BITS,
                          const SerialStreamBuf::FlowControlEnum flowControlType = SerialStreamBuf::DEFAULT_FLOW_CONTROL ) ;

            /** Create a new SerialStream object but do not open it. The
                Open() method will need to be called explicitly on the
                object to communicate with the serial port.

            */
            explicit SerialStream() ;
      
            /** The destructor. It closes the stream associated with
                mFileDescriptor. The rest is done by the fstream destructor.
	  
            */
            virtual ~SerialStream() ; 
            //@}

            /** @name Other Public Methods
             */
            //@{
            /** Open the serial port associated with the specified
                filename, and the specified mode, mode.

            */
            void Open( const std::string fileName, 
                       std::ios_base::openmode openMode = 
                       std::ios_base::in | std::ios_base::out) ;

            /** Close the serial port. No communications can occur with the
                serial port after calling this routine.

            */
            void Close() ;

            /** Returns true if the Stream is in a good open state,
             * false otherwise
             */
            const bool IsOpen() const ;

            /** 
             * Set the baud rate for serial communications. 
             */
            void SetBaudRate(SerialStreamBuf::BaudRateEnum baudRate ) ;

            /** Get the current baud rate being used for serial
                communication. This routine queries the serial port for its
                current settings and returns the baud rate that is being
                used by the serial port. 
	  
                @return The current baud rate for the serial port.
                Note: this is not a constant function because it
                checks to see that it is dealing with a SerialStream
                with a non-null buffer.  If the buffer is null, it
                attempts to set the state of the stream accordingly.
            */
            const SerialStreamBuf::BaudRateEnum BaudRate() ;

            /** Set the character size associated with the serial port. 
	  
            @param size The character size will be set to this value. 
            */
            void SetCharSize(const SerialStreamBuf::CharSizeEnum charSize ) ;

            /** Get the character size being used for serial communication. 
	 
            @return The current character size. 
            */
            const SerialStreamBuf::CharSizeEnum CharSize() ;

            /** Set the number of stop bits used during serial
                communication. The only valid values are 1 and 2.

                @param stop_bits The number of stop bits. (1 or 2). 
	  
            */
            void SetNumOfStopBits(short numOfStopBits) ;

            /** Get the number of stop bits being used during serial
                communication.
	  
                @return The number of stop bits.  */
            const short NumOfStopBits() ; 

            /** Set the parity for serial communication.
	  
            @param parity The parity value. 
	  
            */
            void SetParity(const SerialStreamBuf::ParityEnum parityType) ;

            /** Get the current parity setting for the serial port. 
	  
            @return The parity setting for the serial port. 
	  
            */
            const SerialStreamBuf::ParityEnum Parity() ;

            /** Use the specified flow control. 

            */
            void 
            SetFlowControl(const SerialStreamBuf::FlowControlEnum flowControlType) ;

            /** Return the current flow control setting. 

            */
            const SerialStreamBuf::FlowControlEnum FlowControl() ;

            /** Set character buffer size.
                
            */
            const short SetVMin( short vtime ) ;


            /** Get current size of character buffer.
                Look <A HREF="http://www.unixwiz.net/techtips/termios-vmin-vtime.html">here</A>
                for more documentation about VTIME and VMIN.
                
            */
            const short VMin() ;

            /** Set character buffer timing in 10th of a second.
                
            */
            const short SetVTime( short vtime ) ;

            /** Get current timing of character buffer in 10th of a second.
                Look <A HREF="http://www.unixwiz.net/techtips/termios-vmin-vtime.html">here</A>
                for more documentation about VTIME and VMIN.
                
            */
            const short VTime() ;

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
            /* ------------------------------------------------------------
             * Protected Methods
             * ------------------------------------------------------------
             */
        private:
            /* ------------------------------------------------------------
             * Private Data Members
             * ------------------------------------------------------------
             */
            //
            // The copy constructor and the assignment operator are declared
            // but never defined. This allows the compiler to catch any
            // attempts to copy instances of this class.
            //
            SerialStream( const SerialStream& ) ;
            SerialStream& operator=( const SerialStream& ) ;

            /** The SerialStreamBuf object that will be used by the stream
                to communicate with the serial port.

            */
            SerialStreamBuf *mIOBuffer ;

            /* ----------------------------------------------------------------
             * Private Methods
             * ----------------------------------------------------------------
             */
            /* Set the serial port to ignore the modem status lines. If the
               specified boolean parameter is false then the meaning of
               this function is reversed i.e. the serial port will start
               using the modem status lines.

               @param ignore If true then the modem status lines will be
               ignored otherwise they will be used during the
               communication.

            */
            //void IgnoreModemStatusLines(bool ignore=true) ;

            /* Enable the serial port receiver. This will allow us to read
               data from the serial port.
       
               @param enable If true then the received will be
               enabled. Otherwise it will be disabled.

            */
            //void EnableReceiver(bool enable=true) ;

        } ; // class SerialStream

        inline
        SerialStream::SerialStream() : 
          std::iostream(0), mIOBuffer(0) {
            //
            // Close the stream
            //
            Close() ;
        }

        inline
        SerialStream::~SerialStream() {
            // 
            // If a SerialStreamBuf is associated with this SerialStream
            // then we need to destroy it here.
            //
            if( mIOBuffer ) {
                delete mIOBuffer ;
            }
        }

        inline
        void 
        SerialStream::Close() {
            //
            // If a SerialStreamBuf is associated with the SerialStream then
            // destroy it.
            //
            if( mIOBuffer ) {
                delete mIOBuffer ;
                mIOBuffer = 0 ;
            }
        }

        inline
        const bool
        SerialStream::IsOpen() const {
            //
            // Checks to see if mIOBuffer is a null buffer, if not,
            // calls the is_open() function on this streams SerialStreamBuf,
            // mIOBuffer
            //
            if ( ! mIOBuffer ) {
                return false ;
            }
            return mIOBuffer->is_open() ;
        }

    } // namespace LibSerial
} // extern "C++"
#endif // #ifndef _SerialStream_h_
