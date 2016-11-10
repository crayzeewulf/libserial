/******************************************************************************
 *   @file PosixSignalDispatcher.cpp                                          *
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
 
#include "PosixSignalDispatcher.h"
#include "PosixSignalHandler.h"

#include <cstring>
#include <map>
#include <signal.h>


namespace
{
    /**
     * @brief Implementation class for the PosixSignalDispatcher.
     */
    class PosixSignalDispatcherImpl
    {
    public:
        /**
         * @brief As in the case of PosixSignalDispatcher, this is also
         *        a singleton class. The only instance of this class can
         *        be obtained using this method.
         */
        static
        PosixSignalDispatcherImpl&
        Instance();

        /**
         * @brief Implementation of PosixSignalDispatcher::AttachHandler()
         */
        void
        AttachHandler(const int&          posixSignalNumber,
                      PosixSignalHandler& signalHandler)
            throw(PosixSignalDispatcher::CannotAttachHandler);

        /**
         * @brief Implementation of PosixSignalDispatcher::DetachHandler()
         */
        void
        DetachHandler(const int&                posixSignalNumber,
                      const PosixSignalHandler& signalHandler)
            throw(PosixSignalDispatcher::CannotDetachHandler,
                  std::logic_error);
    private:
        /**
         * @brief List of signal handlers that are currently associated
         *        with the dispatcher.
         */
        typedef std::multimap<int, PosixSignalHandler*> SignalHandlerList;
        static SignalHandlerList mSignalHandlerList;

        /**
         * @brief List of signal handlers that were originally attached
         *        to the corresponding signals.
         */
        typedef std::map<int, struct sigaction> OriginalSigactionList;
        static OriginalSigactionList mOriginalSigactionList;

        /**
         * @brief Default constructor.
         */
        PosixSignalDispatcherImpl();

        /**
         * @brief Destuctor.
         */
        ~PosixSignalDispatcherImpl();

        /**
         * @brief Static function that is used to attach the signal
         *        dispatcher to a signal using sigaction().
         */
        static
        void
        SigactionHandler(int signalNumber);
    };

    // Initialization of static members of class PosixSignalDispatcherImpl.
    PosixSignalDispatcherImpl::SignalHandlerList
    PosixSignalDispatcherImpl::mSignalHandlerList;

    PosixSignalDispatcherImpl::OriginalSigactionList
    PosixSignalDispatcherImpl::mOriginalSigactionList;

}

PosixSignalDispatcher::PosixSignalDispatcher()
{
    /* Empty */
}

PosixSignalDispatcher::~PosixSignalDispatcher()
{
    /* Empty */
}

PosixSignalDispatcher&
PosixSignalDispatcher::Instance()
{
    static PosixSignalDispatcher single_instance;
    return single_instance;
}

void
PosixSignalDispatcher::AttachHandler(const int&          posixSignalNumber,
                                     PosixSignalHandler& signalHandler)
    throw(CannotAttachHandler)
{
    PosixSignalDispatcherImpl::Instance().AttachHandler(posixSignalNumber,
                                                        signalHandler);
    return;
}

void
PosixSignalDispatcher::DetachHandler(const int&                posixSignalNumber,
                                     const PosixSignalHandler& signalHandler)
    throw(CannotDetachHandler,
          std::logic_error)
{
    PosixSignalDispatcherImpl::Instance().DetachHandler(posixSignalNumber,
                                                        signalHandler);
}

namespace
{
    inline
    PosixSignalDispatcherImpl::PosixSignalDispatcherImpl()
    {
        /* empty */
    }

    inline
    PosixSignalDispatcherImpl::~PosixSignalDispatcherImpl()
    {
        /* empty */
    }

    inline
    PosixSignalDispatcherImpl&
    PosixSignalDispatcherImpl::Instance()
    {
        static PosixSignalDispatcherImpl single_instance;
        return single_instance;
    }

    inline
    void
    PosixSignalDispatcherImpl::AttachHandler(const int&          posixSignalNumber,
                                             PosixSignalHandler& signalHandler)
    throw (PosixSignalDispatcher::CannotAttachHandler)
    {
        // Attach this instance of PosixSignalDispatcher to the specified signal.
        struct sigaction sigaction_info;
        sigaction_info.sa_handler = PosixSignalDispatcherImpl::SigactionHandler;
        sigemptyset(&sigaction_info.sa_mask);
        sigaction_info.sa_flags = 0;
        
        // Install the handler and get a copy of the previous handler.
        struct sigaction old_action;
        
        if ( sigaction(posixSignalNumber,
                       &sigaction_info,
                       &old_action ) < 0)
        {
            throw PosixSignalDispatcher::CannotAttachHandler(strerror(errno));
        }
        
        // Save a copy of the old handler if it is not PosixSignalDispatcher::SignalHandler.
        if (PosixSignalDispatcherImpl::SigactionHandler != old_action.sa_handler)
        {
            mOriginalSigactionList.insert(
                OriginalSigactionList::value_type(posixSignalNumber,
                                                  old_action));
        }
        
        // Add the specified handler to the list of handlers associated with the signal.
        mSignalHandlerList.insert(SignalHandlerList::value_type(posixSignalNumber,
                                                                &signalHandler));
        return;
    }

    inline
    void
    PosixSignalDispatcherImpl::DetachHandler(const int&                posixSignalNumber,
                                             const PosixSignalHandler& signalHandler)
    throw(PosixSignalDispatcher::CannotDetachHandler,
          std::logic_error)
    {
        // Get the range of values in the SignalHandlerList corresponding
        // to the specified signal number.
        std::pair<SignalHandlerList::iterator, SignalHandlerList::iterator>
            iterator_range = mSignalHandlerList.equal_range(posixSignalNumber);
        
        // Check if signalHandler is attached to the posixSignalNumber signal.
        SignalHandlerList::iterator sig_handler_location = mSignalHandlerList.end();
        
        for(SignalHandlerList::iterator i = iterator_range.first; i != iterator_range.second; ++i)
        {
            if (i->second == &signalHandler)
            {
                sig_handler_location = i;
                break;
            }
        }
        
        // If the signal handler is found, we need to remove it from the list.
        if (mSignalHandlerList.end() != sig_handler_location)
        {
            mSignalHandlerList.erase(sig_handler_location);
            
            // Remove the signal dispatcher from handling the signal and replace the original
            // signal if this was the only signal handler associated with the specified signal number.
            if (0 == mSignalHandlerList.count(posixSignalNumber))
            {
                //Retrieve the original sigaction corresponding to the signal.
                OriginalSigactionList::iterator original_sigaction =
                    mOriginalSigactionList.find(posixSignalNumber);
                
                // Throw an exception if the signal dispatcher implementation is incorrect
                if (mOriginalSigactionList.end() == original_sigaction)
                {
                    throw std::logic_error("Signal dispatcher in invalid state.");
                }
                
                // Throw an exception if any errors are encountered replacing the original handler.
                if (sigaction(posixSignalNumber,
                              &original_sigaction->second,
                              NULL ) < 0)
                {
                    throw PosixSignalDispatcher::CannotDetachHandler(strerror(errno));
                }
            }
        }
        return;
    }

    inline
    void
    PosixSignalDispatcherImpl::SigactionHandler(int signalNumber)
    {
        // If we got a signal other than SIGIO here then throw an exception. 
        if (signalNumber != SIGIO)
        {
            std::string error_msg; 
            error_msg += "Invalid or unexpected signal: ";
            error_msg += signalNumber;
            throw std::runtime_error(error_msg);
        }

        // Get a list of handlers associated with signalNumber.
        std::pair<SignalHandlerList::iterator, SignalHandlerList::iterator>
            iterator_range = mSignalHandlerList.equal_range(signalNumber);
        
        // Call each handler.
        for(SignalHandlerList::iterator i = iterator_range.first; i != iterator_range.second; ++i)
        {
            i->second->HandlePosixSignal(signalNumber);
        }

        return;
    }
}
