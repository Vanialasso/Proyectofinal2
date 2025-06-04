/*
 * sc_channel_queue.h
 *
 *  Created on: Apr 27, 2024
 *
 *      Author: Oscar Rodriguez Polo
 */

/****************************************************************************
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; either version 2
 *   of the License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,USA.
 *
 *
 ****************************************************************************/


#ifndef LLSW_SC_CHANNEL_DRV_INCLUDE_SC_CHANNEL_QUEUE_H_
#define LLSW_SC_CHANNEL_DRV_INCLUDE_SC_CHANNEL_QUEUE_H_

#include "public/config.h"
#include "public/basic_types.h"

/**
 * \brief	Init TC Queue.*/

void TCQueue_Init(void (*lockFunction)(void), void (*unlockFunction)(void));

/**
 * \brief	Return true if TCQueueIsFull */

bool TCQueue_IsFull();

/**
 * \brief	Return true if TCQueueIsEmpty*/

bool TCQueue_IsEmpty();


/**
 * \brief	Get the memory of the Tail TC */
uint8_t* TCQueue_GetTailTCMemory();

/**
 * \brief	Tail TC is Completed, update Tail*/
void TCQueue_TailTCCompleted(uint16_t length);

/**
 * \brief	Get the memory of the Head TC */

uint8_t* TCQueue_GetHeadTCMemory(uint16_t &headtclength);

/**
 * \brief	Head TC has been extracted, update head*/

void TCQueue_HeadTCExtracted();

#endif /* LLSW_SC_CHANNEL_DRV_INCLUDE_SC_CHANNEL_QUEUE_H_ */
