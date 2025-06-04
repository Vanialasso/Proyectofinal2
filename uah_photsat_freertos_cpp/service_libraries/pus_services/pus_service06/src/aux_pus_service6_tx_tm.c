/*
 * aux_pus_service6_tx_tm.c
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

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "pus_service6/aux_pus_service6_tc_exec.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service06/include/public/pus_service6.h"
#include "../../pus_service06/include/pus_service6/aux_pus_service6_tc_exec.h"
#include "../../pus_service06/include/pus_service6/aux_pus_service6_utils.h"

error_code_t pus_service6_tx_TM_6_6(uint8_t MemID, uint8_t *pdata,
		uint32_t address, uint16_t length) {

	error_code_t error = 0;

	tm_handler_t tm_handler;

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//MemID=1 -> TM
		error = tm_handler_append_uint8_appdata_field(&tm_handler, MemID);

		//N=1 -> TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler, 1);

		//Address-> TM
		error += tm_handler_append_uint32_appdata_field(&tm_handler, address);

		//length-> TM
		error += tm_handler_append_uint16_appdata_field(&tm_handler, length);

		//Data -> TM
		for (uint16_t i = 0; i < length; i++) {
			error += tm_handler_append_uint8_appdata_field(&tm_handler,
					*(pdata + i));
		}

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 6, 6);
		}
		//free memory
		tm_handler_free_mem(&tm_handler);
	}

	return error;

}

error_code_t pus_service6_tx_TM_6_10(uint8_t MemID,uint32_t address,
		uint16_t length, uint16_t checksum) {

	error_code_t error = 0;

	tm_handler_t tm_handler;

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//MemID=1 -> TM
		error = tm_handler_append_uint8_appdata_field(&tm_handler, MemID);

		//N=1 -> TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler, 1);

		//Address-> TM
		error += tm_handler_append_uint32_appdata_field(&tm_handler, address);

		//length-> TM
		error += tm_handler_append_uint16_appdata_field(&tm_handler, length);

		//checksum-> TM
		error += tm_handler_append_uint16_appdata_field(&tm_handler, checksum);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 6, 10);
		}
		//free memory
		tm_handler_free_mem(&tm_handler);
	}

	return error;

}
