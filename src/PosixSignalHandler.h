/******************************************************************************
 *   @file PosixSignalHandler   .h                                            *
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

#ifndef _PosixSignalHandler_h_
#define _PosixSignalHandler_h_

/**
 * @brief Gets a method called when the corresponding signal is received.
 *        A PosixSignalHandler must be connected to the PosixSignalDispatcher
 *        for it to be called.
 */
class PosixSignalHandler
{
public:
    /**
     * @brief This method is called when the specified POSIX signal is 
     *        received by the PosixSignalDispatcher that is managing
     *        this handler.
     */
    virtual void HandlePosixSignal(const int signalNumber) = 0;
     
    /**
     * @brief Destructor is declared virtual as we expect this class to be
     *        subclassed. It is also declared pure abstract to make this
     *        class a pure abstract class.
     */
    virtual ~PosixSignalHandler() = default;
};

#endif // #ifndef _PosixSignalHandler_h_
