/*
 * Time-stamp: <00/08/10 23:12:35 pagey>
 *
 * $Id: SerialStream.h,v 1.1.1.1 2000-08-17 09:30:21 pagey Exp $ 
 *
 *
 */
#ifndef SerialStream_h
#define SerialStream_h

#include <string>
#include <fstream>
#include <assert.h>
#include "SerialStreamBuf.h"

extern "C++" {
  namespace LibSerial {
    /** A stream class for accessing serial ports on POSIX operating
	systems. A lot of the functionality of this class has been
	obtained by looking at the code of libserial package by Linas
	Vepstas (linas@linas.org) and the excellent document on serial
	programming by Michael R. Sweet. This document can be found at
	<a href="http://www.easysw.com/~mike/serial/serial.html">
	http://www.easysw.com/~mike/serial/serial.html</a>. The libserial 
	package can be found at <a href="http://www.linas.org/serial/">
	http://www.linas.org/serial/</a>. This class
	allows one to set various parameters of a serial port and then
	access it like a simple fstream. In fact, that is exactly what
	it does. It sets the parameters of the serial port by
	maintaining a file descriptor for the port and uses the
	basic_fstream functions for the IO. We have not implemented
	any file locking yet but it will be added soon. 

	Make sure you read the documentation of the standard fstream
	template before using this class because most of the
	functionality is inherited from fstream. Also a lot of
	information about the various system calls used in the
	implementation can also be found in the Single Unix
	Specification (Version 2). A copy of this document can be
	obtained from <a href="http://www.UNIX-systems.org/">
	http://www.UNIX-systems.org/</a>. We will refer to this
	document as SUS-2.
	
	@author $Author: pagey $ <A HREF="pagey@drcsdca.com">Manish P. Pagey</A>
	@version $Id: SerialStream.h,v 1.1.1.1 2000-08-17 09:30:21 pagey Exp $
     
    */
    class SerialStream : public iostream {
    public:
      /** @name Typedefs
       */
      //@{

      //@}


      /** @name Enumerations
       */
      //@{

      //@}

      /* ----------------------------------------------------------------------
       * Public Static Members
       * ---------------------------------------------------------------------- */
      /** @name Public static members. 
       */
      //@{

      //@}

      /** @name Exceptions
       */
      //@{

      //@}

      /** @name Constructors and Destructor
       */
      //@{

      /** This constructor takes a filename and an openmode to
	  construct a SerialStream object. This results in a call to
	  basic_fstream::open(s,mode). This is the only way to
	  contruct an object of this class. We have to enforce this
	  instead of providing a default constructor because we want
	  to get a file descriptor whenever the basic_fstream::open()
	  function is called. However, this function is not made
	  virtual in the STL hence it is probably not very safe to
	  overload it. We may decide to overload it later but the
	  users of this class will have to make sure that this class
	  is not used as an fstream class. 

	  If the constructor has problems opening the serial port or
	  getting the file-descriptor for the port, it will set the
	  failbit for the stream. So, one must make sure that the
	  stream is in a good state before using it for any further
	  I/O operations.

	  @param filename The filename of the serial port. 
	  @param mode     The openmode for the serial port file. 

       */
      explicit SerialStream( const string       filename, 
			     ios_base::openmode mode = ios::in|ios::out) ;

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
      /** Open the serial port associated with the specified filename,
	  s and the specified mode, mode.

       */
      void Open(const string filename, ios_base::openmode mode = ios_base::in | ios_base::out) ;

      /** Close the serial port. No communications can occur with the
	  serial port after calling this routine.

       */
      void Close() ;

      /** Set the baud rate for serial communications. 

       */
      void SetBaudRate(SerialStreamBuf::BaudRateEnum baud_rate) ;

      /** Get the current baud rate being used for serial
	  communication. This routine queries the serial port for its
	  current settings and returns the baud rate that is being
	  used by the serial port. 
	  
	  @return The current baud rate for the serial port.  
	  
      */
      const SerialStreamBuf::BaudRateEnum BaudRate() ;

      /** Set the character size associated with the serial port. 
	  
	  @param size The character size will be set to this value. 
       */
      void SetCharSize(const SerialStreamBuf::CharSizeEnum size) ;

      /** Get the character size being used for serial communication. 
	 
	  @return The current character size. 
       */
      const SerialStreamBuf::CharSizeEnum CharSize() ;

      /** Set the number of stop bits used during serial
	  communication. The only valid values are 1 and 2.

	  @param stop_bits The number of stop bits. (1 or 2). 
	  
       */
      void SetNumOfStopBits(short stop_bits) ;

      /** Get the number of stop bits being used during serial
	  communication.
	  
	  @return The number of stop bits.  */
      const short NumOfStopBits() ; 

      /** Set the parity for serial communication.
	  
	  @param parity The parity value. 
	  
      */
      void SetParity(const SerialStreamBuf::ParityEnum parity) ;

      /** Get the current parity setting for the serial port. 
	  
	  @return The parity setting for the serial port. 
	  
      */
      const SerialStreamBuf::ParityEnum Parity() ;

      /** Use the specified flow control. 

       */
      void
      SetFlowControl(const SerialStreamBuf::FlowControlEnum flow_c) ;

      /** Return the current flow control setting. 

       */
      const SerialStreamBuf::FlowControlEnum 
      FlowControl() ;

      //@}

      /** @name Operators
       */
      //@{

      //@}

      /* ----------------------------------------------------------------------
       * Friends
       * ----------------------------------------------------------------------
       */
    protected:
      /* ----------------------------------------------------------------------
       * Protected Data Members
       * ----------------------------------------------------------------------
       */
      /* ----------------------------------------------------------------------
       * Protected Methods
       * ----------------------------------------------------------------------
       */
    private:
      /* ----------------------------------------------------------------------
       * Private Data Members
       * ----------------------------------------------------------------------
       */
      /** The SerialStreamBuf object that will be used by the stream
	  to communicate with the serial port.

       */
      SerialStreamBuf *mIOBuffer ;

      /* ----------------------------------------------------------------------
       * Private Methods
       * ----------------------------------------------------------------------
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
      mIOBuffer(0), iostream(0) {
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
    SerialStream::Open(const string filename, ios_base::openmode mode) {
      //
      // Create a new SerialStreamBuf if one does not exist. 
      //
      if( ! mIOBuffer ) {
	this->rdbuf(mIOBuffer=new SerialStreamBuf) ;
	assert(mIOBuffer!=0) ;
      }
      //
      // Open the serial port. 
      //
      if( 0 == mIOBuffer->open(filename, mode) ) {
	setstate(badbit) ;    
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

  } ; // namespace LibSerial
} // extern "C++"
#endif // #ifndef SerialStream_h
