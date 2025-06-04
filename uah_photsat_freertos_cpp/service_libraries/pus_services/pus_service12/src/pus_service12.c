/*
 * pus_service12.c
 *
 *  Created on: Oct 27, 2024
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

#include "public/pus_service12.h"

#include "pus_service12/aux_pus_service12_x_utils.h"
#include "pus_service12/aux_pus_service12_exec_tc.h"

param_monitoring_config_t Param_Monitor_Config[PUS_SERVICE12_MAX_NUM_OF_PMONS];

param_monitoring_transition_t monitoring_transitions[PUS_SERVICE12_MAX_NUM_OF_TRANS];

void pus_service12_startup() {

	for (uint16_t i = 0; i < PUS_SERVICE12_MAX_NUM_OF_PMONS; i++) {

		Param_Monitor_Config[i].type = MonitorFree;

	}

}

/**
 * \brief executes a TC[12,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service12_exec_tc(tc_handler_t *ptc_handler) {

	switch (ptc_handler->tc_df_header.subtype) {
	case (1):
		pus_service12_exec_TC_12_1(ptc_handler);
		break;
	case (2):
		pus_service12_exec_TC_12_2(ptc_handler);
		break;
	case (5):
		pus_service12_exec_TC_12_5(ptc_handler);
		break;
	case (6):
		pus_service12_exec_TC_12_6(ptc_handler);
		break;
	default:
		break;
	}

}

uint16_t pus_service12_get_PMON_COUNT() {

	return PUS_SERVICE12_MAX_NUM_OF_PMONS;

}

bool_t pus_service12_is_PMON_enabled(uint16_t PMONID) {

	bool_t enabled = false;

	if (PMONID <= PUS_SERVICE12_MAX_NUM_OF_PMONS) {

		if (Param_Monitor_Config[PMONID].type != MonitorFree) {

			enabled = Param_Monitor_Config[PMONID].enabled;
		}
	}

	return enabled;
}

monitor_type_t pus_service12_get_PMON_type(uint16_t PMONID) {

	monitor_type_t type = MonitorFree;

	if (PMONID <= PUS_SERVICE12_MAX_NUM_OF_PMONS) {

		type = Param_Monitor_Config[PMONID].type;
	}

	return type;

}

bool_t pus_service12_do_param_limits_monitoring(uint16_t PMONID,
		uint16_t *p_validEvID, param_out_of_limits_info_t *pfault_info) {

	bool_t event_triggered = false;

	if (PMONID < PUS_SERVICE12_MAX_NUM_OF_PMONS) {

		param_monitoring_config_t *p_mon_config;
		p_mon_config = &Param_Monitor_Config[PMONID];

		if ((MonitorCheckTypeLimits == p_mon_config->type)
				&& (p_mon_config->enabled)) {

			//increase interval_ctrl;
			p_mon_config->interval_ctrl++;

			if (p_mon_config->interval_ctrl >= p_mon_config->interval) {

				p_mon_config->interval_ctrl = 0;

				if (pus_service12_PID_is_out_of_high_limit(p_mon_config->PID,
						&p_mon_config->monitor_def.limit_check_def)) {

					//Check if event is triggered
					if (pus_service12_mng_param_out_of_high_limit(PMONID,
							p_mon_config, p_validEvID, pfault_info)) {

						event_triggered = true;

					}
				} else if (pus_service12_PID_is_out_of_low_limit(
						p_mon_config->PID,
						&p_mon_config->monitor_def.limit_check_def)) {

					//Check if event is triggered
					if (pus_service12_mng_param_out_of_low_limit(PMONID,
							p_mon_config, p_validEvID, pfault_info)) {

						event_triggered = true;

					}

				} else {

					monitor_status_t new_status;
					monitor_status_t current_status;
					data_pool_item_t item_value;

					new_status.limit_status = MonitorLimitsWithinLimits;

					sys_data_pool_get_param_value(p_mon_config->PID,
							&item_value);

					current_status = p_mon_config->current_status;

					if (MonitorLimitsAboveHighLimit
							== current_status.limit_status) {

						//No event triggered, but status must be updated
						pus_service12_mng_new_status(PMONID, &item_value,
								&p_mon_config->monitor_def.limit_check_def.high_limit,
								p_mon_config, new_status);
					} else {

						//No event triggered, but status must be updated
						pus_service12_mng_new_status(PMONID, &item_value,
								&p_mon_config->monitor_def.limit_check_def.low_limit,
								p_mon_config, new_status);

					}

				}

			}

		}

	}
	return event_triggered;
}

bool_t pus_service12_do_param_check_value_monitoring(uint16_t PMONID,
		uint16_t *p_EvID, param_value_fault_info_t *pfault_info) {

	bool_t event_triggered = false;

	if (PMONID < PUS_SERVICE12_MAX_NUM_OF_PMONS) {

		param_monitoring_config_t *p_mon_config;
		p_mon_config = &Param_Monitor_Config[PMONID];

		if (MonitorCheckTypeExpectedValue
				== Param_Monitor_Config[PMONID].type) {

			//increase interval_ctrl;
			p_mon_config->interval_ctrl++;

			if (p_mon_config->interval_ctrl >= p_mon_config->interval) {

				p_mon_config->interval_ctrl = 0;

				//if Value is different from expected
				if (!pus_service12_PID_has_expected_masked_value(
						p_mon_config->PID,
						&p_mon_config->monitor_def.value_check_def)) {

					//Check if event is triggered
					if (pus_service12_mng_param_unexpected_value(PMONID,
							p_mon_config, p_EvID, pfault_info)) {
						event_triggered = true;

					}
				} else {

					monitor_status_t new_status;
					data_pool_item_t item_value;

					new_status.value_status = MonitorExpValueExpectedValue;

					sys_data_pool_get_param_value(p_mon_config->PID,
							&item_value);


					//No event triggered, but status must be updated
					pus_service12_mng_new_status(PMONID, &item_value,
							&p_mon_config->monitor_def.value_check_def.expected_value,
							p_mon_config, new_status);

				}
			}
		}
	}
	return event_triggered;

}

