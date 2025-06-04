/*
 * aux_pus_service4_utils.c
 *
 *  Created on: Nov 20, 2024
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

#include "../../pus_service04/include/pus_service4/aux_pus_service4_utils.h"

#include "public/obt_drv.h"
#include "public/pus_sys_data_pool.h"
#include "pus_services/aux_pus_services_utils.h"
#include "../../pus_service04/include/public/pus_service4.h"

stats_def_t ParamStats[PUS_SERVICE4_MAX_NUM_OF_STATS];

stats_config_t StatsConfig = { { false, false, false, false }, { 0, 0, 0, 0 }, {
		0, 0, 0, 0 } };

error_code_t pus_service4_add_PID_stats(uint16_t param_id) {

	error_code_t error = 1;

	uint16_t pid_index;

	//Check if PID statistics is defined
	error = pus_service4_get_PID_defined_stats_index(param_id, &pid_index);

	if (error) {

		//If not, find free index
		error = pus_service4_get_free_index(&pid_index);

	}

	if (!error) {

		StatsConfig.PID[pid_index] = param_id;
		StatsConfig.defined[pid_index] = true;
		//always reset when enable
		pus_service4_reset_stats_index(pid_index);

	}

	return error;

}

error_code_t pus_service4_delete_PID_stats(uint16_t param_id) {

	error_code_t error = 1;

	uint16_t pid_index;

	//Check if PID statistics is defined
	error = pus_service4_get_PID_defined_stats_index(param_id, &pid_index);

	if (!error) {
		StatsConfig.defined[pid_index] = false;
	}

	return error;

}

error_code_t pus_service4_delete_all_stats() {

	error_code_t error = 0;

	for (uint8_t i = 0; i < PUS_SERVICE4_MAX_NUM_OF_STATS; i++) {

		StatsConfig.defined[i] = false;

	}
	return error;

}

bool_t pus_service4_is_stats_index_defined(uint16_t stats_index) {

	bool_t is_defined = false;

	if (PUS_SERVICE4_MAX_NUM_OF_STATS > stats_index) {

		if (StatsConfig.defined[stats_index]) {
			//PID must be valid
			if (sys_data_pool_is_PID_valid(StatsConfig.PID[stats_index])) {
				is_defined = true;
			} else {
				//not defined if PID is not valid
				StatsConfig.defined[stats_index] = false;
			}
		}
	}

	return is_defined;
}

bool_t pus_service4_is_PID_stats_defined(uint16_t param_id) {

	error_code_t error = 1;

	uint16_t pid_index;

	//Check if PID statistics is defined
	error = pus_service4_get_PID_defined_stats_index(param_id, &pid_index);

	return (0 == error);
}

uint8_t pus_service4_total_stats_defined() {

	uint8_t counter = 0;
	for (uint16_t i = 0; (i < PUS_SERVICE4_MAX_NUM_OF_STATS); i++) {
		if (pus_service4_is_stats_index_defined(i)) {
			counter++;

		}
	}
	return counter;
}

error_code_t pus_service4_get_PID_defined_stats_index(uint16_t param_id,
		uint16_t *p_validindex) {

	error_code_t error = 1;

	if (p_validindex) {

		for (uint16_t i = 0; ((i < PUS_SERVICE4_MAX_NUM_OF_STATS) && error);
				i++) {

			if (param_id == StatsConfig.PID[i]) {
				if (StatsConfig.defined[i]) {

					if (sys_data_pool_is_PID_valid(param_id)) {
						*p_validindex = i;
						error = 0;
					} else {

						StatsConfig.defined[i] = false;
					}
				}

			}
		}

	}

	else {
		error = 2;
	}

	return error;

}

error_code_t pus_service4_get_free_index(uint16_t *p_validindex) {

	error_code_t error = 1;

	if (p_validindex) {
		for (uint16_t i = 0; ((i < PUS_SERVICE4_MAX_NUM_OF_STATS) && error);
				i++) {
			//Its free if not defined or PID is not valid
			if ((!StatsConfig.defined[i])
					|| (!sys_data_pool_is_PID_valid(StatsConfig.PID[i]))) {

				*p_validindex = i;
				error = 0;

			}
		}
	} else {

		error = 2;
	}

	return error;
}

error_code_t pus_service4_reset_stats_index(uint16_t pid_index) {

	error_code_t error = 1;

	if (pid_index < PUS_SERVICE4_MAX_NUM_OF_STATS) {

		ParamStats[pid_index].samples = 0;
		StatsConfig.start_time[pid_index] = obt_drv_get_current_obt();
		error = 0;
	}

	return error;

}

error_code_t pus_service4_update_PID_stats(uint16_t pid_index) {

	error_code_t error = 0;

	if ((pid_index < PUS_SERVICE4_MAX_NUM_OF_STATS)
			&& StatsConfig.defined[pid_index]) {

		uint16_t param_id = StatsConfig.PID[pid_index];

		if (ParamStats[pid_index].samples > 0) {

			//update max
			if (sys_data_pool_param_higher_than_limit(param_id,
					&ParamStats[pid_index].max)) {

				sys_data_pool_get_param_value(param_id,
						&ParamStats[pid_index].max);

				ParamStats[pid_index].max_obt = obt_drv_get_current_obt();

			} else if (sys_data_pool_param_lower_than_limit(param_id,
					&ParamStats[pid_index].min)) {

				sys_data_pool_get_param_value(param_id,
						&ParamStats[pid_index].min);

				ParamStats[pid_index].min_obt = obt_drv_get_current_obt();

			}
			//update mean

			sys_data_pool_param_update_mean(param_id,
					&ParamStats[pid_index].mean_value,
					ParamStats[pid_index].samples);

		} else {

			sys_data_pool_get_param_value(param_id, &ParamStats[pid_index].max);

			sys_data_pool_get_param_value(param_id, &ParamStats[pid_index].min);

			sys_data_pool_get_param_value(param_id,
					&ParamStats[pid_index].mean_value);

			ParamStats[pid_index].max_obt = obt_drv_get_current_obt();
			ParamStats[pid_index].min_obt = ParamStats[pid_index].max_obt;

		}
		//New sample done
		ParamStats[pid_index].samples++;

	} else {

		error = 1;
	}

	return error;
}

