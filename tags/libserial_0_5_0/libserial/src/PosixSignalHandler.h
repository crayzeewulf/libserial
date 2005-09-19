#ifndef _PosixSignalHandler_h_
#define _PosixSignalHandler_h_

/**
 * Gets a method called when the corresponding signal is received.
 * A PosixSignalHandler must be connected to the PosixSignalDispatcher
 * for it to be called.
 */
class PosixSignalHandler
{
public:
    /**
     * This method is called when the specified POSIX signal is 
     * received by the PosixSignalDispatcher that is managing
     * this handler.
     */
    virtual void HandlePosixSignal( int signalNumber ) = 0 ;
     
    /**
     * Destructor is declared virtual as we expect this class to be
     * subclassed. It is also declared pure abstract to make this
     * class a pure abstract class.
     */
    virtual ~PosixSignalHandler() = 0 ;
} ;

inline
PosixSignalHandler::~PosixSignalHandler()
{
    /* empty */
}
#endif // #ifndef _PosixSignalHandler_h_
