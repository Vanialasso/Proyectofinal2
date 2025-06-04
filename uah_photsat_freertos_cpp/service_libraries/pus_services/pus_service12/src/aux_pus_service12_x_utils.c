/*
 * aux_pus_service12_x_utils.c
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

#include "pus_service12/aux_pus_service12_x_utils.h"
#include "pus_service12/aux_pus_service12_tx_tm.h"
#include "public/obt_drv.h"

extern param_monitoring_config_t Param_Monitor_Config[PUS_SERVICE12_MAX_NUM_OF_PMONS];

extern param_monitoring_transition_t monitoring_transitions[PUS_SERVICE12_MAX_NUM_OF_TRANS];

uint16_t Monitor_Transition_Counter = 0;

void pus_service12_exec_TC_12_2_set_unchecked(uint16_t validPMONID) {

	//do nothing if it is not a valid.
	if (validPMONID < PUS_SERVICE12_MAX_NUM_OF_PMONS) {

		//Cannot disable it if it is not in the monitor list (is undefined)
		switch (Param_Monitor_Config[validPMONID].type) {
		case (MonitorCheckTypeLimits):

			Param_Monitor_Config[validPMONID].current_status.limit_status =
					MonitorLimitsUnchecked;
			break;

		case (MonitorCheckTypeExpectedValue):

			Param_Monitor_Config[validPMONID].current_status.value_status =
					MonitorExpValueUnchecked;
			break;
		default:

			break;
		}

		//temp_status = status;
		Param_Monitor_Config[validPMONID].temp_status =
				Param_Monitor_Config[validPMONID].current_status;

	}

}

bool_t pus_service12_PID_is_out_of_high_limit(uint16_t PID,
		const param_limit_check_definition_t *const p_check_def) {

	return sys_data_pool_param_higher_than_limit(PID, &p_check_def->high_limit);

}

bool_t pus_service12_PID_is_out_of_low_limit(uint16_t PID,
		const param_limit_check_definition_t *const p_check_def) {

	return sys_data_pool_param_lower_than_limit(PID, &p_check_def->low_limit);
}

bool_t pus_service12_PID_has_expected_masked_value(uint16_t PID,
		const param_value_check_definition_t *const p_check_def) {

	return sys_data_pool_masked_param_equal_than_masked_item(PID,
			&p_check_def->expected_value, &p_check_def->mask_value);

}

bool_t pus_service12_are_status_equal(monitor_type_t type,
		const monitor_status_t status1, const monitor_status_t status2) {

	bool_t are_equal = false;

	switch (type) {

	case (MonitorCheckTypeLimits):
		are_equal = (status1.limit_status == status2.limit_status);
		break;
	case (MonitorCheckTypeExpectedValue):
		are_equal = (status1.value_status == status2.value_status);
		break;
	default:
		are_equal = false;
	}

	return are_equal;

}

bool_t pus_service12_mng_param_out_of_high_limit(uint16_t pmon_id,
		param_monitoring_config_t *p_mon_config, uint16_t *p_EvID,
		param_out_of_limits_info_t *pfault_info) {

	bool_t event_triggered = false;

	//Check pointers are valid
	if (p_mon_config && p_EvID && pfault_info) {

		monitor_status_t new_status;

		new_status.limit_status = MonitorLimitsAboveHighLimit;

		pfault_info->PID = p_mon_config->PID;

		pfault_info->PIDLimit =
				p_mon_config->monitor_def.limit_check_def.high_limit;

		sys_data_pool_get_param_value(p_mon_config->PID,
				&pfault_info->PIDValue);

		//a transition means event must be triggered
		if (pus_service12_mng_new_status(pmon_id, &pfault_info->PIDValue,
				&pfault_info->PIDLimit, p_mon_config, new_status)) {

			event_triggered = true;

			*p_EvID = p_mon_config->monitor_def.limit_check_def.high_limit_EvID;

		}

	}
	return event_triggered;
}

bool_t pus_service12_mng_param_out_of_low_limit(uint16_t pmon_id,
		param_monitoring_config_t *p_mon_config, uint16_t *p_EvID,
		param_out_of_limits_info_t *pfault_info) {

	bool_t event_triggered = false;

	//Check pointers are valid
	if (p_mon_config && p_EvID && pfault_info) {

		monitor_status_t new_status;

		new_status.limit_status = MonitorLimitsBelowLowLimit;

		pfault_info->PID = p_mon_config->PID;

		pfault_info->PIDLimit =
				p_mon_config->monitor_def.limit_check_def.low_limit;

		sys_data_pool_get_param_value(p_mon_config->PID,
				&pfault_info->PIDValue);

		//event must be triggered
		if (pus_service12_mng_new_status(pmon_id, &pfault_info->PIDValue,
				&pfault_info->PIDLimit, p_mon_config, new_status)) {

			event_triggered = true;

			*p_EvID = p_mon_config->monitor_def.limit_check_def.low_limit_EvID;

		}

	}
	return event_triggered;
}

bool_t pus_service12_mng_param_unexpected_value(uint16_t pmon_id,
		param_monitoring_config_t *p_mon_config, uint16_t *p_EvID,
		param_value_fault_info_t *pfault_info) {

	bool_t event_triggered = false;

	//Check pointers are valid
	if (p_mon_config && p_EvID && pfault_info) {

		monitor_status_t new_status;

		new_status.value_status = MonitorExpValueUnexpectedValue;

		pfault_info->PID = p_mon_config->PID;

		pfault_info->PIDExpectedValue =
				p_mon_config->monitor_def.value_check_def.expected_value;

		pfault_info->PIDMask =
				p_mon_config->monitor_def.value_check_def.mask_value;

		sys_data_pool_get_param_value(p_mon_config->PID,
				&pfault_info->PIDValue);

		//event must be triggered
		if (pus_service12_mng_new_status(pmon_id, &pfault_info->PIDValue,
				&pfault_info->PIDExpectedValue, p_mon_config, new_status)) {

			event_triggered = true;

			*p_EvID = p_mon_config->monitor_def.value_check_def.EvID;

		}

	}
	return event_triggered;

}

bool_t pus_service12_mng_new_status(uint16_t pmon_id,
		const data_pool_item_t *const p_item_value,
		const data_pool_item_t *const p_limit_value,
		param_monitoring_config_t *p_mon_config, monitor_status_t new_status) {

	bool_t transition = false;

//do nothing if p_mon_config is not valid.
	if (p_mon_config) {

		//if new_status is different than current_status
		//and tmp_status == new_status

		if (!pus_service12_are_status_equal(p_mon_config->type,
				p_mon_config->current_status, new_status)) {

			//new status is the same than temp
			if (pus_service12_are_status_equal(p_mon_config->type,
					p_mon_config->temp_status, new_status)) {

				p_mon_config->repetition_ctrl++;

			} else {

				//first time the new status is reached
				//OBT must be annotated

				p_mon_config->temp_status = new_status;
				p_mon_config->repetition_ctrl = 1;
				p_mon_config->transition_obt = obt_drv_get_current_obt();
			}

			//Check if repetition_ctrl is equal to repetition
			if (p_mon_config->repetition_ctrl >= p_mon_config->repetition) {

				pus_service12_add_monitoring_transition(pmon_id, p_item_value,
						p_limit_value, p_mon_config,
						p_mon_config->current_status, new_status);

				p_mon_config->repetition_ctrl = 0;

				p_mon_config->current_status = new_status;

				transition = true;
			}

		} else {
			//new_status is the same than current,
			//update temp_status to current
			p_mon_config->temp_status = new_status;
			p_mon_config->repetition_ctrl = 0;
		}

	}

	return transition;
}

bool_t pus_service12_mng_is_valid_check_limit_def(uint16_t PID,
		const param_limit_check_definition_t *const p_limit_check_def) {

	data_pool_item_type_t type = sys_data_pool_item_type(PID);

	return sys_data_pool_item1_higher_than_item2(type,
			&p_limit_check_def->high_limit, &p_limit_check_def->low_limit);
}

error_code_t pus_service12_add_valid_mng_mon_def(uint16_t pmon_id,
		const param_monitoring_config_t *const p_mon_config) {

	error_code_t error = 1;

	if (p_mon_config) {

		Param_Monitor_Config[pmon_id] = *p_mon_config;

		Param_Monitor_Config[pmon_id].enabled = false;

		pus_service12_exec_TC_12_2_set_unchecked(pmon_id);

		error = 0;

	}

	return error;
}

error_code_t pus_service12_exec_TC_12_X_get_PMON_limit_check_def(
		tc_handler_t *ptc_handler, uint16_t PID,
		param_limit_check_definition_t *p_limit_check_def) {

	error_code_t error = 0;

	uint16_t param_size;

	data_pool_item_type_t type;

	//the serialized param bytes are get from the TC
	uint8_t param_serialized_bytes[sizeof(data_pool_item_t)];

	type = sys_data_pool_item_type(PID);

	param_size = sys_data_pool_item_type_size(type);

	//get the low limit serialized_bytes
	error += tc_handler_get_byte_array_appdata_field(ptc_handler,
			param_serialized_bytes, param_size);

	//Set low limit
	sys_data_pool_set_item_from_serialized_bytes(type, param_serialized_bytes,
			&p_limit_check_def->low_limit);

	//Get low limit EvID

	error += tc_handler_get_uint16_appdata_field(ptc_handler,
			&p_limit_check_def->low_limit_EvID);

	//get the high limit serialized_bytes
	error += tc_handler_get_byte_array_appdata_field(ptc_handler,
			param_serialized_bytes, param_size);

	sys_data_pool_set_item_from_serialized_bytes(type, param_serialized_bytes,
			&p_limit_check_def->high_limit);

	//Get high limit EvID

	error += tc_handler_get_uint16_appdata_field(ptc_handler,
			&p_limit_check_def->high_limit_EvID);

	return error;
}

error_code_t pus_service12_exec_TC_12_X_get_PMON_value_check_def(
		tc_handler_t *ptc_handler, uint16_t PID,
		param_value_check_definition_t *p_value_check_def) {

	error_code_t error = 0;

	uint16_t param_size;

	data_pool_item_type_t type;

	//the serialized param bytes are get from the TC
	uint8_t param_serialized_bytes[sizeof(data_pool_item_t)];

	type = sys_data_pool_item_type(PID);

	param_size = sys_data_pool_item_type_size(type);

	//get the mask serialized_bytes
	error += tc_handler_get_byte_array_appdata_field(ptc_handler,
			param_serialized_bytes, param_size);

	//Set mask
	sys_data_pool_set_item_from_serialized_bytes(type, param_serialized_bytes,
			&p_value_check_def->mask_value);

	//get the expected value serialized_bytes
	error += tc_handler_get_byte_array_appdata_field(ptc_handler,
			param_serialized_bytes, param_size);

	//Set expected value
	sys_data_pool_set_item_from_serialized_bytes(type, param_serialized_bytes,
			&p_value_check_def->expected_value);

	//Get EvID

	error += tc_handler_get_uint16_appdata_field(ptc_handler,
			&p_value_check_def->EvID);

	return error;
}

error_code_t pus_service12_add_monitoring_transition(uint16_t pmon_id,
		const data_pool_item_t *const p_new_value,
		const data_pool_item_t *const p_limit_value,
		param_monitoring_config_t *p_mon_config, monitor_status_t prev_status,
		monitor_status_t new_status) {

	error_code_t error=1;

	//pointers must be valid
	if (p_new_value && p_limit_value && p_mon_config) {

		error=0;

		if (PUS_SERVICE12_MAX_NUM_OF_TRANS > Monitor_Transition_Counter) {

			monitoring_transitions[Monitor_Transition_Counter].PID =
					p_mon_config->PID;

			monitoring_transitions[Monitor_Transition_Counter].PMONID = pmon_id;

			monitoring_transitions[Monitor_Transition_Counter].type =
					p_mon_config->type;

			monitoring_transitions[Monitor_Transition_Counter].limit_value =
					*p_limit_value;

			monitoring_transitions[Monitor_Transition_Counter].new_value =
					*p_new_value;

			if (p_mon_config->type == MonitorCheckTypeExpectedValue) {

				monitoring_transitions[Monitor_Transition_Counter].mask_value =
						p_mon_config->monitor_def.value_check_def.mask_value;
			}

			monitoring_transitions[Monitor_Transition_Counter].new_status =
					new_status;
			monitoring_transitions[Monitor_Transition_Counter].prev_status =
					prev_status;

			monitoring_transitions[Monitor_Transition_Counter].trans_obt =
					p_mon_config->transition_obt;

			//Increase counter
			Monitor_Transition_Counter++;
		}

		if (PUS_SERVICE12_MAX_NUM_OF_TRANS == Monitor_Transition_Counter) {

			pus_service12_tx_TM_12_12(Monitor_Transition_Counter);

			Monitor_Transition_Counter = 0;
		}

	}

	return error;

}
