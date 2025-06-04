/*
 * pus_service5.c
 *
 *  Created on: Oct 25, 2024
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

#include "../../pus_service05/include/public/pus_service5.h"

#include "../../pus_service05/include/pus_service5/aux_pus_service5_exec_tc.h"
#include "../../pus_service05/include/pus_service5/aux_pus_service5_tx_tm.h"
#include "../../pus_service05/include/pus_service5/aux_pus_service5_utils.h"




ev_type_t pus_service5_get_EvID_type(uint16_t EvID) {

	ev_type_t event_type = EvNotValid;
	int auxType = EvID >> 12;
	int auxID = EvID & 0xFFF;

	switch (auxType) {
	case (1):
		if (auxID <= PUS_5_InformativeEvents) {
			event_type = EvTypeInformative;
		}
		break;
	case (2):
		if (auxID <= PUS_5_LowSeverityAnomalyEvents) {
			event_type = EvTypeLowSevAnomaly;
		}
		break;
	case (3):
		if (auxID <= PUS_5_MediumSeverityAnomalyEvents) {
			event_type = EvTypeMediumSevAnomaly;
		}
		break;
	case (4):
		if (auxID <= PUS_5_HighSeverityAnomalyEvents) {
			event_type = EvTypeHighSevAnomaly;
		}

		break;

	default:
		event_type = EvNotValid;
	}

	return event_type;

}

bool_t pus_service5_is_EvID_valid(uint16_t EvID) {

	ev_type_t EvIDType = pus_service5_get_EvID_type(EvID);

	return (EvNotValid != EvIDType);
}

bool_t pus_service5_is_EvID_enabled(uint16_t EvID) {

	bool_t enabled = false;

	if (pus_service5_is_EvID_valid(EvID)) {

		enabled=pus_service5_check_EvID_enable_flag(EvID);
	}

	return enabled;

}

void pus_service5_exec_tc(tc_handler_t *ptc_handler) {

	switch (ptc_handler->tc_df_header.subtype) {
	case (5):
		pus_service5_exec_TC_5_5(ptc_handler);
		break;
	case (6):
		pus_service5_exec_TC_5_6(ptc_handler);
		break;
	default:
		break;
	}
}

error_code_t pus_service1_tx_TM_5_1_start_up() {

	uint16_t EvID = TM_5_1_START_UP;

	return pus_service5_tx_TM_5_X_no_aux_data(EvID);
}


error_code_t pus_service1_tx_TM_5_X_param_out_of_limit(uint16_t EvID,
		param_out_of_limits_info_t *pfault_info) {

	error_code_t error = 1;

	//Only if EvID is enabled and pfault_info is not NULL
	if (pus_service5_check_EvID_enable_flag(EvID)&& pfault_info) {

		tm_handler_t tm_handler;

		error = pus_service5_tx_TM_5_X_with_aux_data_prologue(EvID,
				&tm_handler);

		//Error if no memory has been allocated
		if (0 == error) {

			//PID -> TM
			error = tm_handler_append_uint16_appdata_field(&tm_handler,
					pfault_info->PID);

			//PIDValue -> TM
			error += tm_handler_append_data_pool_item(&tm_handler,
					pfault_info->PID,&pfault_info->PIDValue);

			//PIDLimit -> TM
			error += tm_handler_append_data_pool_item(&tm_handler,
					pfault_info->PID,&pfault_info->PIDLimit);

			if (0 == error) {
				pus_service5_tx_TM_5_X_with_aux_data_epilogue(EvID,
						&tm_handler);
			}

			//free memory
			tm_handler_free_mem(&tm_handler);
		}

	}
	return error;

}

error_code_t pus_service1_tx_TM_5_X_param_check_value_fail(uint16_t EvID
		,param_value_fault_info_t *pfault_info){

	error_code_t error = 1;

	//Only if EvID is enabled and pfault_info is not NULL
	if (pus_service5_check_EvID_enable_flag(EvID)&& pfault_info) {

		tm_handler_t tm_handler;

		error = pus_service5_tx_TM_5_X_with_aux_data_prologue(EvID,
				&tm_handler);

		//Error if no memory has been allocated
		if (0 == error) {

			//PID -> TM
			error = tm_handler_append_uint16_appdata_field(&tm_handler,
					pfault_info->PID);

			//PIDValue -> TM
			error += tm_handler_append_data_pool_item(&tm_handler,
					pfault_info->PID,&pfault_info->PIDValue);

			//PIDMask -> TM
			error += tm_handler_append_data_pool_item(&tm_handler,
					pfault_info->PID,&pfault_info->PIDMask);

			//PIDExpected -> TM
			error += tm_handler_append_data_pool_item(&tm_handler,
					pfault_info->PID,&pfault_info->PIDExpectedValue);

			if (0 == error) {
				pus_service5_tx_TM_5_X_with_aux_data_epilogue(EvID,
						&tm_handler);
			}

			//free memory
			tm_handler_free_mem(&tm_handler);
		}

	}
	return error;

}

