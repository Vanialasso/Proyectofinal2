/*
 * aux_pus_service4_exec_tc.c
 *
 *  Created on: Nov 21, 2024
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

#include "../../pus_service04/include/pus_service4/aux_pus_service4_exec_tc.h"

#include "public/pus_sys_data_pool.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "pus_services/aux_pus_services_utils.h"
#include "public/obt_drv.h"
#include "public/pus_sys_data_pool.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service04/include/public/pus_service4.h"
#include "../../pus_service04/include/pus_service4/aux_pus_service4_utils.h"

void pus_service4_exec_TC_4_1(tc_handler_t *ptc_handler) {

	error_code_t error = 0;

	pus_service1_tx_TM_1_3(ptc_handler);

	for (uint16_t i = 0; (i < PUS_SERVICE4_MAX_NUM_OF_STATS) && (!error); i++){

		if (StatsConfig.defined[i]) {

			tm_handler_t tm_handler;

			//Alloc memory
			error += tm_handler_mem_alloc_and_startup(&tm_handler);

			if (0 == error) {

				uint8_t N; //N is 1
				uint32_t start_time, current_time;
				uint16_t PID;

				start_time = StatsConfig.start_time[i];

				PID = StatsConfig.PID[i];

				current_time = obt_drv_get_current_obt();

				StatsConfig.start_time[i] = current_time;

				N = 1; //N is 1

				//start_time -> TM
				error = tm_handler_append_uint32_appdata_field(&tm_handler,
						start_time);

				//current_time -> TM
				error += tm_handler_append_uint32_appdata_field(&tm_handler,
						current_time);

				//N -> TM
				error += tm_handler_append_uint8_appdata_field(&tm_handler, N);


				//PID-> TM
				error += tm_handler_append_uint16_appdata_field(&tm_handler,
						PID);

				//Samples-> TM
				error += tm_handler_append_uint32_appdata_field(&tm_handler,
						ParamStats[i].samples);

				//Max -> TM
				error += pus_services_TM_X_Y_write_saved_PIDValue(&tm_handler,
						PID, &ParamStats[i].max);

				//Max OBT
				error += tm_handler_append_uint32_appdata_field(&tm_handler,
						ParamStats[i].max_obt);

				//Min -> TM
				error += pus_services_TM_X_Y_write_saved_PIDValue(&tm_handler,
						PID, &ParamStats[i].min);

				//Min OBT
				error += tm_handler_append_uint32_appdata_field(&tm_handler,
						ParamStats[i].min_obt);

				//Mean -> TM
				error += pus_services_TM_X_Y_write_saved_PIDValue(&tm_handler,
						PID, &ParamStats[i].mean_value);

				pus_service4_reset_stats_index(i);

				//close and Tx only in no error
				if (0 == error) {

					error += tm_handler_close_and_tx(&tm_handler, 4, 2);

				}

				//free memory
				tm_handler_free_mem(&tm_handler);
			}
		}

	}

	if (!error) {

		pus_service1_tx_TM_1_7(ptc_handler);

	} else {

		;

	}
}

void pus_service4_exec_TC_4_6(tc_handler_t *ptc_handler) {

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
			if (sys_data_pool_is_PID_valid(PID)) {

				pus_service1_tx_TM_1_3(ptc_handler);
				//Try to add the stats of this param
				error = pus_service4_add_PID_stats(PID);

				if (!error) {
					pus_service1_tx_TM_1_7(ptc_handler);

				} else {

					pus_service1_tx_TM_1_8_not_free_stats_config(ptc_handler, PID);
				}

			} else {

				//If not valid PID
				pus_service1_tx_TM_1_4_PID_not_valid(ptc_handler, PID);

			}
		}
//free memory
		tc_handler_free_memory(ptc_handler);

	}
}

void pus_service4_exec_TC_4_7(tc_handler_t *ptc_handler) {

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
			if (sys_data_pool_is_PID_valid(PID)) {

				pus_service1_tx_TM_1_3(ptc_handler);


				//Try to delete the stats of this param
				error = pus_service4_delete_PID_stats(PID);

				if (!error) {
					pus_service1_tx_TM_1_7(ptc_handler);

				} else {

					pus_service1_tx_TM_1_8_PID_stats_undefined(ptc_handler, PID);
				}
			} else {	//If not valid PID

				pus_service1_tx_TM_1_4_PID_not_valid(ptc_handler, PID);

			}

		}
	}
//free memory
	tc_handler_free_memory(ptc_handler);

}

