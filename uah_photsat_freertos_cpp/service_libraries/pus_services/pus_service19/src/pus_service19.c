/*
 * pus_service19.c
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

#include "public/pus_service19.h"
#include "pus_service19/aux_pus_service19_utils.h"
#include "pus_service19/aux_pus_service19_exec_tc.h"

void pus_service19_mng_event_action(uint16_t EvID) {

	bool_t is_defined;
	bool_t is_enabled;

	ev_action_id_t ev_action_id;

	//Get event action info if it exists
	is_defined=pus_service19_is_ev_action_defined(EvID,
			&ev_action_id, &is_enabled);

	//If defined and enabled
	if(is_defined&&is_enabled){

		pus_service19_add_ev_action_to_queue(EvID,ev_action_id);
	}

}

uint8_t pus_service19_pending_ev_actions() {

	return (pus_service19_pending_ev_5_1_actions()
			+ pus_service19_pending_ev_5_2_actions()
			+ pus_service19_pending_ev_5_3_actions()
			+ pus_service19_pending_ev_5_4_actions());
}

error_code_t pus_service19_extract_next_ev_action(tc_handler_t *ptc_handler) {

	error_code_t error=0; //No error
	if (ptc_handler) {

		//priority order 5_4 -> 5_3 -> 5_3 -> 5_1
		if(pus_service19_pending_ev_5_4_actions()){

			error=pus_service19_extract_ev_5_4_action(ptc_handler);

		}else if (pus_service19_pending_ev_5_3_actions()){

			error=pus_service19_extract_ev_5_3_action(ptc_handler);

		}else if (pus_service19_pending_ev_5_3_actions()){

			error=pus_service19_extract_ev_5_2_action(ptc_handler);

		}else if (pus_service19_pending_ev_5_1_actions()){

			error=pus_service19_extract_ev_5_1_action(ptc_handler);

		}else{

			error=1;
			ptc_handler->raw_tc_mem_descriptor.tc_num_bytes = 0;
			ptc_handler->raw_tc_mem_descriptor.p_tc_bytes = NULL;
		}
	}else{

		error=1;
	}

	return error;
}

void pus_service19_exec_tc(tc_handler_t *ptc_handler){
	switch (ptc_handler->tc_df_header.subtype) {
	case (1):
		pus_service19_exec_TC_19_1(ptc_handler);
		break;
	case (2):
		pus_service19_exec_TC_19_2(ptc_handler);
		break;
	case (4):
		pus_service19_exec_TC_19_4(ptc_handler);
		break;
	case (5):
		pus_service19_exec_TC_19_5(ptc_handler);
		break;

	default:
		break;
	}

}


