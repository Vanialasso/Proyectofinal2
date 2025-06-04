/*
 * sc_channel_drv_v1.h
 *
 *  Created on: Apr 27, 2024
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

#ifndef PUBLIC__SC_CHANNEL_DRV_IFACE_V1_H
#define PUBLIC__SC_CHANNEL_DRV_IFACE_V1_H



#include "public/config.h"
#include "public/basic_types.h"
#include "public/tmtc_dyn_mem.h"

#ifdef __cplusplus

#include "public/cdtcmemdescriptor_iface_v1.h"



/**
 * \brief	Rx Data from the SC_Channel.
 * @return true if a complete Telecommand has been received.
 */
bool_t SC_Channel_RxData();
/**
 * \brief	Get Next Telecommand.
 * @param p_tc_descriptor pointer to PUS TC Decriptor.
 */
void SC_Channel_GetNextTC(CDTCMemDescriptor &tcDescriptor);

/**
 * \brief	Handle Pending TCs.
 */
void SC_Channel_HandlePendingTCs();

#endif


#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief	SpaceCraft Channel Init .*/
void SC_Channel_Init();

/**
 * \brief	telemetry transmission
 * @param ptm_descritpor telemetry descriptor pointer
 */
void SC_Channel_TxTM(const tm_mem_descriptor_t *ptm_descriptor);

#ifdef __cplusplus
}
#endif









#endif // PUBLIC__CONSOLE_DRV_IFACE_V1_H
