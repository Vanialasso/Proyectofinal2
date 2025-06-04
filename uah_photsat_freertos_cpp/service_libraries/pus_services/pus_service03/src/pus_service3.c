/*
 * asw_pus_service3.c
 *
 *  Created on: Oct 21, 2024
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

#include "../../pus_service03/include/public/pus_service3.h"
#include "../../pus_service03/include/public/pus_service3.h"

#include <public/pus_sys_data_pool.h>
#include "public/adc_drv.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service03/include/pus_service3/aux_pus_service3_exec_tc.h"
#include "../../pus_service03/include/pus_service3/aux_pus_service3_tx_tm.h"

//TODO Define SID 20 with parameters 12, 30, 31, 32, 33, 13, 14

HK_config_t HKConfig[PUS_SERVICE3_MAX_NUM_OF_SIDS] = {
		{ SIDDisabled, 0, 10, 0, 5, { 0, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDDisabled, 10, 4, 0, 3, { 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDDisabled, 11, 5, 0, 2, { 14, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
		{ SIDConfigUnused, 0, 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } }

};


void pus_service3_do_HK() {


	//Check HK SID generation
	for (int i = 0; i < PUS_SERVICE3_MAX_NUM_OF_SIDS; i++) {
		if (SIDEnabled==HKConfig[i].status) {
			HKConfig[i].interval_ctrl++;
			if (HKConfig[i].interval_ctrl >= HKConfig[i].interval) {

				//Generate SID
				pus_service3_tx_TM_3_25(i);

				HKConfig[i].interval_ctrl = 0;
			}
		}
	}

}

void pus_service3_exec_tc(tc_handler_t *ptc_handler) {
	switch (ptc_handler->tc_df_header.subtype) {
	case (5):
		pus_service3_exec_TC_3_5(ptc_handler);
		break;
	case (6):
		pus_service3_exec_TC_3_6(ptc_handler);
		break;
	case (31):
		pus_service3_exec_TC_3_31(ptc_handler);
		break;
	default:
		break;
	}

}


