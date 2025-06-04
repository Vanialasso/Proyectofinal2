/*
 * aux_pus_service12_tx_tm.c
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


#include "pus_service12/aux_pus_service12_tx_tm.h"

extern param_monitoring_transition_t monitoring_transitions[PUS_SERVICE12_MAX_NUM_OF_TRANS];

error_code_t pus_service12_tx_TM_12_12(uint16_t trans_counter) {

	error_code_t error = 0;


	if (trans_counter <= PUS_SERVICE12_MAX_NUM_OF_TRANS) {

		tm_handler_t tm_handler;

		//Alloc memory
		error = tm_handler_mem_alloc_and_startup(&tm_handler);

		if (0 == error) {

			//N -> TM
			error = tm_handler_append_uint8_appdata_field(&tm_handler,
					trans_counter);

			for (uint8_t i = 0; i < trans_counter; i++) {

				uint8_t aux_prev_status;
				uint8_t aux_new_status;

				//PMONID -> TM
				error = tm_handler_append_uint16_appdata_field(&tm_handler,
						monitoring_transitions[i].PMONID);

				//PID -> TM
				error += tm_handler_append_uint16_appdata_field(&tm_handler,
						monitoring_transitions[i].PID);

				//Type -> TM
				error += tm_handler_append_uint8_appdata_field(&tm_handler,
						(uint8_t)monitoring_transitions[i].type);

				if(MonitorCheckTypeExpectedValue==monitoring_transitions[i].type){

					//Mask Value -> TM

					error += tm_handler_append_data_pool_item(&tm_handler
							,monitoring_transitions[i].PID
							,&monitoring_transitions[i].mask_value);

					aux_prev_status=monitoring_transitions[i].prev_status.value_status;

					aux_new_status=monitoring_transitions[i].new_status.value_status;

				}else{

					aux_prev_status=monitoring_transitions[i].prev_status.limit_status;

					aux_new_status=monitoring_transitions[i].new_status.value_status;

				}

				//Param Value -> TM
				error += tm_handler_append_data_pool_item(&tm_handler
										,monitoring_transitions[i].PID
										,&monitoring_transitions[i].new_value);

				//Limit Value -> TM
				error += tm_handler_append_data_pool_item(&tm_handler
										,monitoring_transitions[i].PID
										,&monitoring_transitions[i].limit_value);

				//Prev Status -> TM
				error += tm_handler_append_uint8_appdata_field(&tm_handler,
						aux_prev_status);

				//Current Status -> TM
				error += tm_handler_append_uint8_appdata_field(&tm_handler,
						aux_new_status);


				//Trans OBT -> TM
				error += tm_handler_append_uint32_appdata_field(&tm_handler,
						monitoring_transitions[i].trans_obt);


			}
			//close and Tx only in no error
			if (0 == error) {

				error = tm_handler_close_and_tx(&tm_handler, 12, 12);
			}
			//free memory
			tm_handler_free_mem(&tm_handler);
		}

	} else {

		error = 2;
	}

	return error;

}
