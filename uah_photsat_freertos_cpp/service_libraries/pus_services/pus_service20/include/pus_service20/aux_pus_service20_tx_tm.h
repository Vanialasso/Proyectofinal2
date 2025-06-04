/*
 * aux_pus_service20_tx_tm.h
 *
 *  Created on: Oct 26, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE20_INCLUDE_PUS_SERVICE20_AUX_PUS_SERVICE20_TX_TM_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE20_INCLUDE_PUS_SERVICE20_AUX_PUS_SERVICE20_TX_TM_H_

#include "public/config.h"

#include "public/pus_tm_handler.h"

/**
 * \brief transmit the TM[20,2] telemetry parameter
 * \param validPID a valid PID identifier
 */
error_code_t pus_service20_tx_TM_20_2(uint16_t validPID);


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE20_INCLUDE_PUS_SERVICE20_AUX_PUS_SERVICE20_TX_TM_H_ */
