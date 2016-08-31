/***************************************************************************
 *   @file PosixSignalDispatcher.h                                         *
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

#ifndef _PosixSignalDispatcher_h_
#define _PosixSignalDispatcher_h_

#include <stdexcept>

/*
 * Forward declarations.
 */
class PosixSignalHandler ;

/**
 * @note The signal dispatcher will not interfere with any signals for which
 *       there are no signal handlers attached. Similarly, if a signal handler
 *       function is already attached to a signal that the dispatcher is asked to
 *       administer, that signal handler will be called after all the attached
 *       PosixSignalHandlers.
 * 
 * @todo Make this a singleton class.
 */
class PosixSignalDispatcher
{
public:
    /**
     * @brief This is a singleton class and there is only one instance of this class
     *        per process. This instance can be obtained using the GetInstance() method.
     */
    static PosixSignalDispatcher& Instance();

    /**
     * @brief Exception thrown when AttachHandler() fails due to a runtime error.
     */
    class CannotAttachHandler : public std::runtime_error
    {
    public:
        CannotAttachHandler(const std::string& whatArg)
            : runtime_error(whatArg)
        {
        }
    };

    /**
     * @brief Exception thrown when DetachHandler() fails due to a runtime error.
     */
    class CannotDetachHandler : public std::runtime_error
    {
    public:
        CannotDetachHandler( const std::string& whatArg )
            : runtime_error(whatArg)
        {
        }
    };

    /**
     * @brief Attaches a signal handler to the signal dispatcher. The signal
     *        handler's HandlePosixSignal() method will be called every time
     *        the specified signal is received. The signal handler should
     *        not be destroyed while it attached to the signal dispatcher.
     *        Otherwise, weird things are bound to happen (i.e. "undefined
     *        behavior" shall ensue). Make sure you call DetachHandler() from
     *        the destructor of the signal handler.
     *
     *        If a PosixSignalHandler is attached to the same signal number
     *        multiple times, it will be called multiple times. Furthermore,
     *        it should also be detached as many times as it was attached
     *        before it is destroyed. Otherwise, undefined behavior may result.
     *
     * @param posixSignalNumber The signal number that will result in
     *        call to the HandlePosixSignal() method of the signal handler.
     * @param signalHandler The signal handler to be invoked on receiving
     *        a posixSignalNumber signal.
     * @throw CannotDetachHandler This exception is thrown if the method cannot detach the handler.
     */
    void AttachHandler(const int posixSignalNumber,
                       PosixSignalHandler& signalHandler)
        throw(CannotAttachHandler);

    /**
     * @brief Detach the specified signal handler from the signal dispatcher.
     *        The signal handler will stop being called on receiving the
     *        corresponding POSIX signal. If the signal handler is not
     *        attached to the signal dispatcher when this method is called
     *        then this method has no effect.
     * @param posixSignalNumber The signal number corresponding to the signal handler.
     * @param signalHandler The signal handler to be detached.
     * @throw CannotDetachHandler This exception is thrown if the method cannot detach the handler.
     * @throw std::logic_error This exception is thrown if any standard logic error is encountered.
     */
    void DetachHandler(const int posixSignalNumber,
                       const PosixSignalHandler& signalHandler)
        throw(CannotDetachHandler, std::logic_error);

private:
    /**
     * @brief This is a singleton class and the only instances of this class
     *        can only be accessed using the Instance() method. This is
     *        enforced by making the default constructor a private member
     *        disalloweing construction of new instances of this class
     */
    PosixSignalDispatcher();

    /**
     * @brief This class cannot be subclassed. We enforce this by making
     *        the destructor a private member.
     */
    ~PosixSignalDispatcher();

    /**
     * @brief Copying of an instance of this class is not allowed. We
     *        enforce this by making the copy constructor and the
     *        assignment operator private members.
     */
    PosixSignalDispatcher(const PosixSignalDispatcher& otherInstance);

    /**
     * @brief Copying of an instance of this class is not allowed. We
     *        enforce this by making the copy constructor and the
     *        assignment operator private members.
     */
    const PosixSignalDispatcher&
    operator=(const PosixSignalDispatcher& otherInstance);
} ;

#endif
