/*
 * aux_pus_service6_tc_exec.c
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE6_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_TC_EXEC_C_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE6_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_TC_EXEC_C_

#include "../../pus_service06/include/pus_service6/aux_pus_service6_tc_exec.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service06/include/public/pus_service6.h"
#include "../../pus_service06/include/pus_service6/aux_pus_service6_tx_tm.h"
#include "../../pus_service06/include/pus_service6/aux_pus_service6_utils.h"

void pus_service6_exec_TC_6_2(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t MemID;
	uint8_t N;
	uint32_t address;
	uint16_t length;

	// TC -> MemID
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &MemID);

	// TC -> N
	error += tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> address
	error += tc_handler_get_uint32_appdata_field(ptc_handler, &address);

	// TC -> length
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &length);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

	} else {

		if (!pus_service6_is_address_valid(MemID, address)) {

			pus_service1_tx_TM_1_4_mem_address_not_valid(ptc_handler, MemID,
					address);

		} else if (!pus_service6_is_address_valid(MemID,
				address + length - 1)) {

			pus_service1_tx_TM_1_4_mem_address_not_valid(ptc_handler, MemID,
					address + length - 1);

		} else if (!pus_service6_is_MemID_WriteEnabled(MemID)) {

			pus_service1_tx_TM_1_4_mem_id_read_only(ptc_handler, MemID);
		} else {

			uint8_t data_bytes[PUS_SERVICE6_MAX_NUM_OF_BYTES_MEM_LOAD];

			//get the param_serialized_bytes
			error = tc_handler_get_byte_array_appdata_field(ptc_handler,
					data_bytes, length);

			if (error) {

				// error in pack length
				pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

			} else { //No error, so execute command

				pus_service1_tx_TM_1_3(ptc_handler);

				//PID is valid, so error is not handled
				error = pus_service6_MemID_write_data(MemID, data_bytes,
						address, length);

				if (!error) {
					pus_service1_tx_TM_1_7(ptc_handler);

				} else {
					pus_service1_tx_TM_1_8_mem_access_error(ptc_handler, MemID);
				}
			}
		}

	}
	//free memory
	tc_handler_free_memory(ptc_handler);
}

void pus_service6_exec_TC_6_5(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t MemID;
	uint8_t N;
	uint32_t address;
	uint16_t length;

	// TC -> MemID
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &MemID);

	// TC -> N
	error += tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> address
	error += tc_handler_get_uint32_appdata_field(ptc_handler, &address);

	// TC -> length
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &length);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

	} else {

		if (error) {

			// error in pack length
			pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		} else {

			if (!pus_service6_is_address_valid(MemID, address)) {

				pus_service1_tx_TM_1_4_mem_address_not_valid(ptc_handler, MemID,
						address);

			} else if (!pus_service6_is_address_valid(MemID,
					address + length - 1)) {

				pus_service1_tx_TM_1_4_mem_address_not_valid(ptc_handler, MemID,
						address + length - 1);

			} else {

				//Start execution
				pus_service1_tx_TM_1_3(ptc_handler);

				//the serialized param bytes are get from the Memory
				uint8_t data_bytes[PUS_SERVICE6_MAX_NUM_OF_BYTES_MEM_DUMP];

				error = pus_service6_MemID_read_data(MemID, data_bytes, address,
						length);

				if (error) {

					pus_service1_tx_TM_1_8_mem_access_error(ptc_handler, MemID);

				} else {

					pus_service6_tx_TM_6_6(MemID, data_bytes, address, length);
				}

			}
		}

	}
	//free memory
	tc_handler_free_memory(ptc_handler);
}

void pus_service6_exec_TC_6_9(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t MemID;
	uint8_t N;
	uint32_t address;
	uint16_t length;

	// TC -> MemID
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &MemID);

	// TC -> N
	error += tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> address
	error += tc_handler_get_uint32_appdata_field(ptc_handler, &address);

	// TC -> length
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &length);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

	} else {

		if (error) {

			// error in pack length
			pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		} else {

			if (!pus_service6_is_address_valid(MemID, address)) {

				pus_service1_tx_TM_1_4_mem_address_not_valid(ptc_handler, MemID,
						address);

			} else if (!pus_service6_is_address_valid(MemID,
					address + length - 1)) {

				pus_service1_tx_TM_1_4_mem_address_not_valid(ptc_handler, MemID,
						address + length - 1);

			} else {

				//Start execution
				pus_service1_tx_TM_1_3(ptc_handler);

				uint16_t checksum;

				error = pus_service6_MemID_check_data(MemID, address, length,
						&checksum);

				if (error) {

					pus_service1_tx_TM_1_8_mem_access_error(ptc_handler, MemID);

				} else {

					pus_service6_tx_TM_6_10(MemID, address, length, checksum);
				}

			}
		}

	}
	//free memory
	tc_handler_free_memory(ptc_handler);
}

#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE6_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_TC_EXEC_C_ */
