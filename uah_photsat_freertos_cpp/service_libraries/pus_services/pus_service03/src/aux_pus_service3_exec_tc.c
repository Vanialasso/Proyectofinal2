/*
 * aux_pus_service3_exec_tc.c
 *
 *  Created on: Oct 24, 2024
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


#include <public/pus_sys_data_pool.h>
#include "public/adc_drv.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"

#include "public/pus_tm_handler.h"

#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service03/include/public/pus_service3.h"
#include "../../pus_service03/include/public/pus_service3.h"
#include "../../pus_service03/include/pus_service3/aux_pus_service3_tx_tm.h"
#include "../../pus_service03/include/pus_service3/aux_pus_service3_utils.h"

extern HK_config_t HKConfig[PUS_SERVICE3_MAX_NUM_OF_SIDS];

void pus_service3_exec_TC_3_5_3_6(tc_handler_t *ptc_handler,
		bool_t enabled_config) {

	error_code_t error;

	uint8_t N;
	uint16_t SID;
	uint8_t SID_index;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> SID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &SID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is valid this SID?
			error += pus_service3_get_SID_index(SID, &SID_index);

			if (!error) {

				pus_service1_tx_TM_1_3(ptc_handler);

				if(enabled_config)
					HKConfig[SID_index].status=SIDEnabled;
				else
					HKConfig[SID_index].status=SIDDisabled;

				HKConfig[SID_index].interval_ctrl = 0;

				pus_service1_tx_TM_1_7(ptc_handler);

			} else {
				//If not valid, SID
				pus_service1_tx_TM_1_4_SID_not_valid(ptc_handler, SID);

			}
		}
	}
	//free memory
	tc_handler_free_memory(ptc_handler);
}

void pus_service3_exec_TC_3_5(tc_handler_t *ptc_handler) {

	pus_service3_exec_TC_3_5_3_6(ptc_handler, true);

}

void pus_service3_exec_TC_3_6(tc_handler_t *ptc_handler) {

	pus_service3_exec_TC_3_5_3_6(ptc_handler, false);

}

void pus_service3_exec_TC_3_31(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t SID;
	uint8_t SID_index;
	uint8_t collection_interval;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> SID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &SID);

	// TC -> collection interval
	error += tc_handler_get_uint8_appdata_field(ptc_handler,
			&collection_interval);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is valid this SID?
			error = pus_service3_get_SID_index(SID, &SID_index);

			if (!error) {

				pus_service1_tx_TM_1_3(ptc_handler);

				HKConfig[SID_index].interval_ctrl = 0;
				HKConfig[SID_index].interval = collection_interval;

				pus_service1_tx_TM_1_7(ptc_handler);

			} else {
				//If not valid, SID
				pus_service1_tx_TM_1_4_SID_not_valid(ptc_handler, SID);

			}
		}
	}
	//free memory
	tc_handler_free_memory(ptc_handler);
}

