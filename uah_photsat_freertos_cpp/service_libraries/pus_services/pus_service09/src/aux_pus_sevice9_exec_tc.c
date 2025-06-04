/*
 * aux_pus_sevice20_exec_tc.c
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

#include "pus_service9/aux_pus_service9_exec_tc.h"

#include "public/pus_service1.h"
#include "public/obt_drv.h"

void pus_service9_exec_TC_9_129(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint32_t next_OBT;

	// TC -> N
	error = tc_handler_get_uint32_appdata_field(ptc_handler, &next_OBT);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		pus_service1_tx_TM_1_3(ptc_handler);
		obt_drv_set_current_obt(next_OBT);
		pus_service1_tx_TM_1_7(ptc_handler);

	}

//free memory
	tc_handler_free_memory(ptc_handler);

}

