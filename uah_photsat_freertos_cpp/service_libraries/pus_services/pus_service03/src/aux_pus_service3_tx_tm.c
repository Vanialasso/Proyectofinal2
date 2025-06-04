/*
 * aux_pus_service3_tx_tm.c
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



#include "../../pus_service03/include/pus_service3/aux_pus_service3_tx_tm.h"

#include "public/pus_sys_data_pool.h"
#include "public/adc_drv.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"

#include "public/pus_tm_handler.h"

#include "pus_services/aux_pus_services_utils.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service03/include/public/pus_service3.h"

extern HK_config_t HKConfig[PUS_SERVICE3_MAX_NUM_OF_SIDS];

error_code_t pus_service3_tx_TM_3_25(uint16_t validSIDIndex) {

	error_code_t error;
	tm_handler_t tm_handler;

	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		error = tm_handler_append_uint16_appdata_field(&tm_handler,
				HKConfig[validSIDIndex].SID);

		for (int j = 0; (j < HKConfig[validSIDIndex].num_of_params) && (!error);
				j++) {

			uint16_t pid = HKConfig[validSIDIndex].param_IDs[j];

			error = pus_services_TM_X_Y_write_PIDValue(&tm_handler, pid);

		}

		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 3, 25);
		}

		tm_handler_free_mem(&tm_handler);

	}

	return error;
}



