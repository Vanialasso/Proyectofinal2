/*
 * aus_pus_service5_tx_tm.h
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE05_INCLUDE_PUS_SERVICE5_AUX_PUS_SERVICE5_TX_TM_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE05_INCLUDE_PUS_SERVICE5_AUX_PUS_SERVICE5_TX_TM_H_

#include <public/pus_tm_handler.h>
#include <public/pus_tc_handler.h>
#include "public/config.h"

/**
 * \brief transmit the TM[5,X] telemetry without auxiliary data
 * \param EvID Event Identifier
 * \return error !=0 if error in tx TM[5,X] with no data
 */
error_code_t pus_service5_tx_TM_5_X_no_aux_data(uint16_t EvID);


/**
* \brief transmition prologue of TM[5,X] telemetry with auxiliary data
* \param EvID Event Identifier
* \param pvalid_tm_handler pointer to a valid tm_handler
* \return error !=0 if no memory allocated in pvalid_tm_handler
*
*/
error_code_t pus_service5_tx_TM_5_X_with_aux_data_prologue(uint16_t EvID
		,tm_handler_t *pvalid_tm_handler);


/**
* \brief transmition epilogue of TM[5,X] telemetry with auxiliary data
* \param EvID Event Identifier
* \param pvalid_tm_handler pointer to a valid tm_handler
* \return error !=0 if error in tx TM[5,X] with aux_data epilogue
*/
error_code_t pus_service5_tx_TM_5_X_with_aux_data_epilogue(uint16_t EvID
		, tm_handler_t *pvalid_tm_handler);


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE05_INCLUDE_PUS_SERVICE5_AUX_PUS_SERVICE5_TX_TM_H_ */
