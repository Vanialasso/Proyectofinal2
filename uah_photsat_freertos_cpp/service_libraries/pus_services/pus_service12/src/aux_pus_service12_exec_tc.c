/*
 * aux_pus_services_12_exec_tc.c
 *
 *  Created on: Oct 28, 2024
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

#include <public/pus_service12.h>
#include <public/pus_sys_data_pool.h>
#include "public/adc_drv.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"

#include "public/pus_service1.h"
#include "public/pus_tm_handler.h"

#include "public/pus_service12.h"
#include "pus_service12/aux_pus_service12_exec_tc.h"
#include "pus_service12/aux_pus_service12_x_utils.h"

//#include "pus_service12/aux_pus_service12_tx_tm.h"

extern param_monitoring_config_t Param_Monitor_Config[PUS_SERVICE12_MAX_NUM_OF_PMONS];

bool_t Param_Monitoring_Active = true;

void pus_service12_exec_TC_12_1(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t PMONID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> PMONID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &PMONID);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

	} else {

		if (PMONID < PUS_SERVICE12_MAX_NUM_OF_PMONS) {

			//Cannot enable it if it is not in the monitor list (is undefined)
			if (Param_Monitor_Config[PMONID].type != MonitorFree) {

				pus_service1_tx_TM_1_3(ptc_handler);

				Param_Monitor_Config[PMONID].enabled = true;
				Param_Monitor_Config[PMONID].interval_ctrl = 0;
				Param_Monitor_Config[PMONID].repetition_ctrl = 0;

				pus_service1_tx_TM_1_7(ptc_handler);

			} else {

				pus_service1_tx_TM_1_4_PMON_undefined(ptc_handler, PMONID);
			}
		} else {

			pus_service1_tx_TM_1_4_PMONID_invalid(ptc_handler, PMONID);
		}

	}

	//free memory
	tc_handler_free_memory(ptc_handler);
}

void pus_service12_exec_TC_12_2(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t PMONID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> PMONID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &PMONID);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

	} else {

		if (PMONID < PUS_SERVICE12_MAX_NUM_OF_PMONS) {

			//Cannot disable it if it is not in the monitor list (is undefined)
			if (Param_Monitor_Config[PMONID].type != MonitorFree) {

				pus_service1_tx_TM_1_3(ptc_handler);

				//Set as unchecked in all types.
				pus_service12_exec_TC_12_2_set_unchecked(PMONID);

				Param_Monitor_Config[PMONID].enabled = false;

				pus_service1_tx_TM_1_7(ptc_handler);

			} else {

				pus_service1_tx_TM_1_4_PMON_undefined(ptc_handler, PMONID);
			}

		} else {

			pus_service1_tx_TM_1_4_PMONID_invalid(ptc_handler, PMONID);
		}
	}

	//free memory
	tc_handler_free_memory(ptc_handler);
}

void pus_service12_exec_TC_12_5(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t PMONID;
	param_monitoring_config_t mon_config;

// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

// TC -> PMONID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &PMONID);

// TC -> PID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &mon_config.PID);

// collect_interval
	error += tc_handler_get_uint8_appdata_field(ptc_handler,
			&mon_config.interval);

// repetition
	error += tc_handler_get_uint8_appdata_field(ptc_handler,
			&mon_config.repetition);

// type
	uint8_t aux;
	error += tc_handler_get_uint8_appdata_field(ptc_handler, &aux);
	mon_config.type = aux;

// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

		//PID must be valid
	} else if (!sys_data_pool_is_PID_valid(mon_config.PID)) {

		//If not valid PID
		pus_service1_tx_TM_1_4_PID_not_valid(ptc_handler, mon_config.PID);

	} else if (mon_config.type == MonitorCheckTypeLimits) {

		error = pus_service12_exec_TC_12_X_get_PMON_limit_check_def(ptc_handler,
				mon_config.PID, &mon_config.monitor_def.limit_check_def);

		if (error) {
			// error in pack length
			pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		} else {

			//Check if pmon check limit definition is valid
			if (pus_service12_mng_is_valid_check_limit_def(mon_config.PID,
					&mon_config.monitor_def.limit_check_def)) {

				pus_service1_tx_TM_1_3(ptc_handler);

				pus_service12_add_valid_mng_mon_def(PMONID, &mon_config);

				pus_service1_tx_TM_1_7(ptc_handler);

			} else {

				//It is not valid. Report with a TM[1,4]
				pus_service1_tx_TM_1_4_PMON_definition_invalid(ptc_handler,
						PMONID);

			}

		}

	} else if (mon_config.type == MonitorCheckTypeExpectedValue) {

		error = pus_service12_exec_TC_12_X_get_PMON_value_check_def(ptc_handler,
				mon_config.PID, &mon_config.monitor_def.value_check_def);

		if (error) {
			// error in pack length
			pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		} else {

			pus_service1_tx_TM_1_3(ptc_handler);

			pus_service12_add_valid_mng_mon_def(PMONID, &mon_config);

			pus_service1_tx_TM_1_7(ptc_handler);

		}

	} else {

		error = pus_service1_tx_TM_1_4_PMON_definition_invalid(ptc_handler,
				PMONID);
	}

	tc_handler_free_memory(ptc_handler);

}

void pus_service12_exec_TC_12_6(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t PMONID;


// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

// TC -> PMONID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &PMONID);

	// Handle error
	if (error) {
		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

		//N must be 1
	} else if (1 != N) {

		error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler, N);

		//PID must be valid
	} else {

		if (MonitorFree == pus_service12_get_PMON_type(PMONID)) {

			//If not valid PMONID
			pus_service1_tx_TM_1_4_PMON_undefined(ptc_handler, PMONID);

		} else if (!Param_Monitor_Config[PMONID].enabled) {

			pus_service1_tx_TM_1_3(ptc_handler);

			Param_Monitor_Config[PMONID].type = MonitorFree;

			pus_service1_tx_TM_1_7(ptc_handler);

		} else {

			pus_service1_tx_TM_1_4_PMON_enabled(ptc_handler, PMONID);
		}

	}

}

