/*
 * aux_pus_service5_tx_tm.c
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

#include "../../pus_service05/include/pus_service5/aux_pus_service5_tx_tm.h"

#include "public/pus_tm_handler.h"
#include "../../pus_service05/include/public/pus_service5.h"
#include "../../pus_service05/include/pus_service5/aux_pus_service5_utils.h"

error_code_t pus_service5_tx_TM_5_X_no_aux_data(uint16_t EvID) {

	error_code_t error = 0;

	tm_handler_t tm_handler;

	//Check if EvID is valid and enabled
	if (pus_service5_is_EvID_enabled(EvID)) {

		//Alloc memory
		error = tm_handler_mem_alloc_and_startup(&tm_handler);

		if (0 == error) {

			//EvID -> TM
			error = tm_handler_append_uint16_appdata_field(&tm_handler, EvID);

			//close and Tx only in no error
			if (0 == error) {

				int8_t subtype = pus_service5_cal_EvID_TM_5_X_subtype(EvID);
				error = tm_handler_close_and_tx(&tm_handler, 5, subtype);
			}
			//free memory
			tm_handler_free_mem(&tm_handler);
		}
	}

	return error;

}

error_code_t pus_service5_tx_TM_5_X_with_aux_data_prologue(uint16_t EvID,
		tm_handler_t *pvalid_tm_handler) {

	error_code_t error = tm_handler_mem_alloc_and_startup(pvalid_tm_handler);

	//Check memory error
	if (0==error) {

		//EvID -> TM no error
		tm_handler_append_uint16_appdata_field(pvalid_tm_handler, EvID);

	}

	return error;
}

error_code_t pus_service5_tx_TM_5_X_with_aux_data_epilogue(uint16_t validEvID,
		tm_handler_t *pvalid_tm_handler) {

	error_code_t error = 0;

	int8_t subtype = pus_service5_cal_EvID_TM_5_X_subtype(validEvID);

	error = tm_handler_close_and_tx(pvalid_tm_handler, 5, subtype);

	return error;
}
