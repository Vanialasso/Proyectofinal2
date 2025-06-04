/*
 * aux_pus_service12_tx_tm.h
 *
 *  Created on: Nov 5, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUS_SERVICE12_AUX_PUS_SERVICE12_TX_TM_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUS_SERVICE12_AUX_PUS_SERVICE12_TX_TM_H_

#include "pus_service12/aux_pus_service12_x_utils.h"

/**
 * \brief transmit the TM[12,12] transition report
 * \param validPID a valid PID identifier
 * \return error !=0 if error in tx TM[12,12]
 *
 */

error_code_t pus_service12_tx_TM_12_12(uint16_t trans_counter);


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUS_SERVICE12_AUX_PUS_SERVICE12_TX_TM_H_ */
