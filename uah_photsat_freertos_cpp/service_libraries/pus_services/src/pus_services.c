/*
 * pus_services.c
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
#include "public/basic_types.h"
#include "public/serialize.h"
#include "public/sc_channel_drv_v1.h"
#include "public/adc_drv.h"
#include "public/watchdog_drv.h"
#include "public/pus_service4.h"
#include "public/pus_service5.h"
#include "public/pus_service12.h"
#include "public/pus_service19.h"
#include <stdlib.h>

void pus_services_startup() {

	SC_Channel_Init();

	init_adc_channel();

	pus_service1_tx_TM_5_1_start_up();

	pus_service4_startup();
	
	pus_service12_startup();

	pus_service129_startup();
}

void pus_services_mng_reboot(){


	watchdog_drv_disable_signal();
}

void pus_services_do_FDIR() {

	//Fault_Detection
	//From each PMONID
	uint16_t number_of_PMONs;

	number_of_PMONs = pus_service12_get_PMON_COUNT();

	for (uint16_t i = 0; i < number_of_PMONs; i++) {

		if (pus_service12_is_PMON_enabled(i)) {

			monitor_type_t type;

			bool_t event_triggered=false;
			bool_t event_enabled=false;
			uint16_t EvID;

			type=pus_service12_get_PMON_type(i);

			switch (type) {
			case (MonitorCheckTypeLimits): {

				param_out_of_limits_info_t fault_info;

				//Event Triggered?
				event_triggered=pus_service12_do_param_limits_monitoring(i,
						&EvID,
						&fault_info);
				if (event_triggered) {

					event_enabled=pus_service5_is_EvID_enabled(EvID);

					if (event_enabled) {

						pus_service1_tx_TM_5_X_param_out_of_limit(EvID,
								&fault_info);


					}
				}
			}
				break;
			case (MonitorCheckTypeExpectedValue): {

				uint16_t EvID;
				param_value_fault_info_t fault_info;

				event_triggered=pus_service12_do_param_check_value_monitoring(i, &EvID,
						&fault_info);

				//Event Triggered?
				if (event_triggered) {

					event_enabled=pus_service5_is_EvID_enabled(EvID);

					if (event_enabled) {

						pus_service1_tx_TM_5_X_param_check_value_fail(EvID,
								&fault_info);


					}
				}
			}
				break;
			default:
				//do nothing
				break;

			}
			//If event triggered && event enabled mng event action

			if(event_triggered && event_enabled)
				pus_service19_mng_event_action(EvID);
		}
	}
}

void pus_services_update_params(){

	//Do asw_update_sys_data_pool
	sample_sys_data_pool_params();

}


bool_t pus_services_sat_in_observation_mode(){

	return (sys_data_pool_get_uint8_param(UAH_SAT_OBS_ModeActive));

}
