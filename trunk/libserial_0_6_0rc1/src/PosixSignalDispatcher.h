//
// C++ Interface: %{MODULE}
//
// Description:
//
//
// Author: %{AUTHOR} <%{EMAIL}>, (C) %{YEAR}
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef _PosixSignalDispatcher_h_
#define _PosixSignalDispatcher_h_

#include <stdexcept>

/*
 * Forward declarations.
 */
class PosixSignalHandler ;

/**

@note The signal dispatcher will not interfere with any signals for which
there are no signal handlers attached. Similarly, if a signal handler
function is already attached to a signal that the dispatcher is asked to
administer, that signal handler will be called after all the attached
PosixSignalHandlers.

@todo Make this a singleton class.

*/
class PosixSignalDispatcher
{
public:
    /**
     * This is a singleton class and there is only one instance of this
     * class per process. This instance can be obtained using the
     * GetInstance() method.
     */
    static PosixSignalDispatcher& Instance() ;

    /**
     * Exception thrown when AttachHandler() fails due to a runtime
     * error.
     */
    class CannotAttachHandler : public std::runtime_error
    {
    public:
        CannotAttachHandler( const std::string& whatArg ) :
            runtime_error(whatArg) { }
    } ;

    /**
     * Exception thrown when DetachHandler() fails due to a runtime
     * error.
     */
    class CannotDetachHandler : public std::runtime_error
    {
    public:
        CannotDetachHandler( const std::string& whatArg ) :
            runtime_error(whatArg) { }
    } ;

    /**
     * Attach a signal handler to the signal dispatcher. The signal
     * handler's HandlePosixSignal() method will be called every time
     * the specified signal is received. The signal handler should
     * not be destroyed while it attached to the signal dispatcher.
     * Otherwise, weird things are bound to happen (i.e. "undefined
     * behavior" shall ensue). Make sure you call DetachHandler() from
     * the destructor of the signal handler.
     *
     * If a PosixSignalHandler is attached to the same signal number
     * multiple times, it will be called multiple times. Furthermore,
     * it should also be detached as many times as it was attached
     * before it is destroyed. Otherwise, undefined behavior may result.
     *
     * @param posixSignalNumber The signal number that will result in
     * call to the HandlePosixSignal() method of the signal handler.
     *
     * @param signalHandler The signal handler to be invoked on receiving
     * a posixSignalNumber signal.
     */
    void AttachHandler( const int           posixSignalNumber,
                        PosixSignalHandler& signalHandler )
        throw( CannotAttachHandler ) ;

    /**
     * Detach the specified signal handler from the signal dispatcher.
     * The signal handler will stop being called on receiving the
     * corresponding POSIX signal. If the signal handler is not
     * attached to the signal dispatcher when this method is called
     * then this method has no effect.
     *
     * @param posixSignalNumber The signal number corresponding to
     * the signal handler.
     *
     * @param signalHandler The signal handler to be detached.
     */
    void DetachHandler( const int                 posixSignalNumber,
                        const PosixSignalHandler& signalHandler )
        throw( CannotDetachHandler,
               std::logic_error ) ;
private:
    /**
     * This is a singleton class and the only instances of this class
     * can only be accessed using the Instance() method. This is
     * enforced by making the default constructor a private member
     * disalloweing construction of new instances of this class
     */
    PosixSignalDispatcher() ;

    /**
     * This class cannot be subclassed. We enforce this by making
     * the destructor a private member.
     */
    ~PosixSignalDispatcher() ;

    /**
     * Copying of an instance of this class is not allowed. We
     * enforce this by making the copy constructor and the
     * assignment operator private members.
     */
    PosixSignalDispatcher( const PosixSignalDispatcher& otherInstance ) ;

    /**
     * Copying of an instance of this class is not allowed. We
     * enforce this by making the copy constructor and the
     * assignment operator private members.
     */
    const PosixSignalDispatcher&
    operator=( const PosixSignalDispatcher& otherInstance ) ;
} ;

#endif
