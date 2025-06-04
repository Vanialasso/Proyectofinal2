/*
 * aux_pus_service6_tx_tm.h
 *
 *  Created on: Nov 26, 2024
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE06_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_TX_TM_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE06_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_TX_TM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "public/config.h"
#include <public/pus_tc_handler.h>

/**
 * \brief transmit the TM[6,6] telemetry parameter
 * \param validPID a valid PID identifier
 */
error_code_t pus_service6_tx_TM_6_6(uint8_t MemID, uint8_t *pdata
		,uint32_t address, uint16_t length);

/**
 * \brief transmit the TM[6,10] telemetry parameter
 * \param validPID a valid PID identifier
 */
error_code_t pus_service6_tx_TM_6_10(uint8_t MemID,
		uint32_t address, uint16_t length, uint16_t checksum);


#ifdef __cplusplus
}
#endif


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE06_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_TX_TM_H_ */
