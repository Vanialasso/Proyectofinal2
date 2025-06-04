/*
 * aux_pus_service5_exec_tc.c
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

#include <public/pus_tc_handler.h>
#include "public/config.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service05/include/public/pus_service5.h"
#include "../../pus_service05/include/pus_service5/aux_pus_service5_utils.h"

void pus_service5_exec_TC_5_5_5_6(tc_handler_t *ptc_handler,
		bool_t enabled_config) {

	error_code_t error;

	uint8_t N;
	uint16_t EvID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> EvID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &EvID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is valid this EvID?

			if (pus_service5_is_EvID_valid(EvID)) {

				pus_service1_tx_TM_1_3(ptc_handler);

				pus_service5_set_EvID_enable_flag(EvID,enabled_config);

				pus_service1_tx_TM_1_7(ptc_handler);

			} else {
				//If not valid, SID
				pus_service1_tx_TM_1_4_EvID_not_valid(ptc_handler, EvID);

			}
		}
	}
	//free memory
	tc_handler_free_memory(ptc_handler);
}

void pus_service5_exec_TC_5_5(tc_handler_t *ptc_handler) {

	pus_service5_exec_TC_5_5_5_6(ptc_handler, true);

}

void pus_service5_exec_TC_5_6(tc_handler_t *ptc_handler) {

	pus_service5_exec_TC_5_5_5_6(ptc_handler, false);

}

