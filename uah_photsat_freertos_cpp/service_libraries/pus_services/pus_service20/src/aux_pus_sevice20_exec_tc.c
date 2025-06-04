/*
 * aux_pus_sevice20_exec_tc.c
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

#include "pus_service20/aux_pus_service20_exec_tc.h"
#include "pus_service20/aux_pus_service20_tx_tm.h"

#include "public/pus_sys_data_pool.h"

#include "public/pus_service1.h"

void pus_service20_exec_TC_20_1(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t PID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> PID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &PID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Check Param validity
			if (!sys_data_pool_is_PID_valid(PID)) {

				//If not valid PID
				pus_service1_tx_TM_1_4_PID_not_valid(ptc_handler, PID);

			} else {

				pus_service1_tx_TM_1_3(ptc_handler);
				pus_service20_tx_TM_20_2(PID);
				pus_service1_tx_TM_1_7(ptc_handler);

			}

		}
	}
//free memory
	tc_handler_free_memory(ptc_handler);

}

void pus_service20_exec_TC_20_3(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t PID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> PID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &PID);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

	} else {

		//Check Param validity
		if (!sys_data_pool_is_PID_valid(PID)) {

			//If not valid PID
			pus_service1_tx_TM_1_4_PID_not_valid(ptc_handler, PID);

		} else {

			if (!sys_data_pool_is_PID_update_via_TC_enabled(PID)) {

				pus_service1_tx_TM_1_4_PID_read_only_via_TC_20_X(ptc_handler, PID);

			} else {

				//the serialized param bytes are get from the TC
				uint8_t param_serialized_bytes[sizeof(data_pool_item_t)];

				//get the param_serialized_bytes
				error = tc_handler_get_byte_array_appdata_field(ptc_handler,
						param_serialized_bytes, sys_data_pool_PID_size(PID));

				if (error) {

					// error in pack length
					pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

				} else { //No error, so execute command

					pus_service1_tx_TM_1_3(ptc_handler);

					//PID is valid, so error is not handled
					sys_data_pool_set_param_from_serialized_bytes(PID,
							param_serialized_bytes);

					pus_service1_tx_TM_1_7(ptc_handler);
				}
			}
		}

	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

