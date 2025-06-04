/*
 * aux_pus_service20_tx_tm.c
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

#include "public/config.h"
#include "public/pus_tm_handler.h"
#include "public/pus_service1.h"

#include "pus_services/aux_pus_services_utils.h"

/**
 * \brief transmit the TM[20,2] telemetry
 * \param validPID a valid PID identifier
 */
error_code_t pus_service20_tx_TM_20_2(uint16_t validPID){

	error_code_t error = 0;

	tm_handler_t tm_handler;

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//N=1 -> TM
		error =tm_handler_append_uint8_appdata_field(&tm_handler,1);

		//PID-> TM
		error += tm_handler_append_uint16_appdata_field(&tm_handler, validPID);

		//PIDValue -> TM
		error += pus_services_TM_X_Y_write_PIDValue(&tm_handler, validPID);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 20, 2);
		}
		//free memory
		tm_handler_free_mem(&tm_handler);
	}

	return error;

}




