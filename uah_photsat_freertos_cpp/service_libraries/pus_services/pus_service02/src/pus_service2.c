/*
 * pus_service2.c
 *
 *  Created on: Nov 14, 2024
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

#include "../../pus_service02/include/public/pus_service2.h"

#include "public/pus_service17.h"
#include "public/ccsds_pus.h"
#include "public/crc.h"

#include "public/pus_tm_handler.h"

#include "public/device_drv.h"
#include "../../pus_service01/include/public/pus_service1.h"

void pus_service2_exec_TC_2_1(tc_handler_t *ptc_handler);

void pus_service2_exec_tc(tc_handler_t *ptc_handler) {
	switch (ptc_handler->tc_df_header.subtype) {
	case (1):
		pus_service2_exec_TC_2_1(ptc_handler);
		break;
	default:
		break;
	}

}

void pus_service2_exec_TC_2_1(tc_handler_t *ptc_handler) {

	uint32_t on_off_address;
	error_code_t error;
	uint8_t N;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> on_off_address
	error += tc_handler_get_uint32_appdata_field(ptc_handler, &on_off_address);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Check On-Off Address
			if (device_drv_is_on_off_address_valid(on_off_address)) {
				//Tx TM[1,3]
				pus_service1_tx_TM_1_3(ptc_handler);

				error = device_drv_on_off(on_off_address);

				if (error) {

					pus_service1_tx_TM_1_8_dev_comman_exec_error(ptc_handler,
							on_off_address);
				} else {
					//Tx TM[1,7]
					pus_service1_tx_TM_1_7(ptc_handler);
				}
			} else {

				pus_service1_tx_TM_1_4_device_address_not_valid(ptc_handler,
						on_off_address);

			}

		}
	}

	//free memory
	tc_handler_free_memory(ptc_handler);
}

