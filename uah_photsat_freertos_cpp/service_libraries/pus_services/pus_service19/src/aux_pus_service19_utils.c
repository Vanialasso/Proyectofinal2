/*
 * aux_pus_service19_utils.c
 *
 *  Created on: Nov 12, 2024
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
#include "public/pus_service19.h"
#include "pus_service19/aux_pus_service19_utils.h"

#include "../../pus_service05/include/public/pus_service5.h"

#define PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS 16
#define PUS_SERVICE19_EVENT_ACTION_QUEUE_DIMENSION 8
#define PUS_SERVICE19_EVENT_ACTION_MAX_NUM_OF_BYTES  	(UAH_PUS_TC_APPDATA_MAX_NUM_OF_BYTES)

struct ev_action_config {
	bool_t enabled;
	uint16_t EvID; //EvID 0 means not used
};

typedef struct ev_action_config ev_action_config_t;

struct ev_action_packet {
	uint8_t ev_action_pack_bytes[PUS_SERVICE19_EVENT_ACTION_MAX_NUM_OF_BYTES];
	uint16_t ev_action_pack_lenght;
};

typedef struct ev_action_packet ev_action_packet_t;

//Configuration of Event Actions. No Actions defined at reset
ev_action_config_t EventActionConfig[PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS] =
		{

		{ false, 0 }, { false, 0 }, { false, 0 }, { false, 0 },
		{ false, 0 }, { false, 0 }, { false, 0 }, { false, 0 },
		{ false, 0 }, { false, 0 }, { false, 0 }, { false, 0 },
		{ false, 0 }, { false, 0 }, { false, 0 }, { false, 0 }

		};

ev_action_packet_t EventActionPacks[PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS];

//One Queue for each Event severity level
ev_action_packet_t PendingActionQueue[4][PUS_SERVICE19_EVENT_ACTION_QUEUE_DIMENSION];
uint8_t PendingActionHead[4] = { 0, 0, 0, 0 };
uint8_t PendingActionNumber[4] = { 0, 0, 0, 0 };

uint8_t pus_service19_pending_ev_5_1_actions() {
	return PendingActionNumber[0];

}

uint8_t pus_service19_pending_ev_5_2_actions() {

	return PendingActionNumber[1];
}

uint8_t pus_service19_pending_ev_5_3_actions() {

	return PendingActionNumber[2];

}

uint8_t pus_service19_pending_ev_5_4_actions() {

	return PendingActionNumber[3];
}



bool_t pus_service19_is_ev_action_defined(uint16_t EvID,
		ev_action_id_t *p_ev_action_id, bool_t *p_enabled) {

	bool_t found = false;

	for (ev_action_id_t i = 0;
			((i < PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS) && !found); i++) {
		if (EvID == EventActionConfig[i].EvID) {
			*p_ev_action_id = i;
			*p_enabled = EventActionConfig[i].enabled;
			found = true;
		}
	}
	return found;
}

error_code_t pus_service19_disable_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id){

	error_code_t error=0;

	if(EvID==EventActionConfig[ev_action_id].EvID){

		EventActionConfig[ev_action_id].enabled=false;

	}else{

		error=1;
	}

	return error;
}

error_code_t pus_service19_enable_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id){

	error_code_t error=0;

	if(EvID==EventActionConfig[ev_action_id].EvID){

		EventActionConfig[ev_action_id].enabled=true;

	}else{

		error=1;
	}

	return error;

}

error_code_t pus_service19_delete_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id){

	error_code_t error=0;

	if(EvID==EventActionConfig[ev_action_id].EvID){

		EventActionConfig[ev_action_id].EvID=0;
		EventActionConfig[ev_action_id].enabled=false;

	}else{

		error=1;
	}

	return error;


}

error_code_t pus_service19_get_free_ev_action_id(
		ev_action_id_t *p_ev_action_id) {
	error_code_t error = 1;
	bool_t found = false;
	for (ev_action_id_t i = 0;
			((i < PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS) && !found); i++) {
		if (0 == EventActionConfig[i].EvID) {
			*p_ev_action_id = i;
			error = 0;
			found=true;
		}
	}
	return error;
}

//	error=pus_service19_get_free_ev_action_index(&ev_action_id);

error_code_t pus_service19_get_ev_action(ev_action_id_t ev_action_id,
		uint8_t *p_action_bytes, uint16_t *p_num_of_bytes) {

	error_code_t error;

	if (!p_action_bytes) {

		error = 1;

	} else if (PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS <= ev_action_id) {

		error = 2;

	} else {

		uint16_t num_of_bytes;

		num_of_bytes = EventActionPacks[ev_action_id].ev_action_pack_lenght;

		*p_num_of_bytes = num_of_bytes;

		for (uint16_t i = 0; i < num_of_bytes; i++) {

			p_action_bytes[i] =
					EventActionPacks[ev_action_id].ev_action_pack_bytes[i];
		}

	}
	return error;

}

error_code_t pus_service19_set_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id,
		uint8_t *p_action_bytes, uint16_t num_of_bytes) {

	error_code_t error;

	if (!p_action_bytes) {

		error = 1;

	} else if (PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS <= ev_action_id) {

		error = 2;

	} else {

		EventActionConfig[ev_action_id].EvID=EvID;

		EventActionPacks[ev_action_id].ev_action_pack_lenght = num_of_bytes;

		for (uint16_t i = 0; i < num_of_bytes; i++) {
			EventActionPacks[ev_action_id].ev_action_pack_bytes[i] =
					p_action_bytes[i];
		}

	}

	return error;

}

error_code_t pus_service19_add_ev_action_to_queue(uint16_t evID,
		ev_action_id_t ev_action_id) {

	error_code_t error = 0;
	ev_type_t event_type;
	uint8_t queue_index = 0;

	if (PUS_SERVICE19_MAX_EVENT_ACTION_DEFINITIONS <= ev_action_id) {

		error = 1;
	} else {

		event_type = pus_service5_get_EvID_type(evID);

		switch (event_type) {

		case (EvTypeInformative):
			queue_index = 0;
			break;
		case (EvTypeLowSevAnomaly):
			queue_index = 1;
			break;
		case (EvTypeMediumSevAnomaly):
			queue_index = 2;
			break;

		case (EvTypeHighSevAnomaly):
			queue_index = 3;
			break;
		default:
			error = 2;
			break;

		}
	}

	if (0 == error) {

		if (PendingActionNumber[queue_index]
				< PUS_SERVICE19_EVENT_ACTION_QUEUE_DIMENSION) {

			uint8_t tail;

			//Get tail Index
			tail = (PendingActionHead[queue_index]
					+ PendingActionNumber[queue_index])
					% PUS_SERVICE19_EVENT_ACTION_QUEUE_DIMENSION;

			PendingActionNumber[queue_index]++; //Update number of elements

			//The action is copied in the queue
			pus_service19_get_ev_action(ev_action_id,
					PendingActionQueue[queue_index][tail].ev_action_pack_bytes,
					&PendingActionQueue[queue_index][tail].ev_action_pack_lenght);

		}else{

			error=3;
		}

	}

	return error;
}

error_code_t pus_service19_extract_ev_action_from_queue(tc_handler_t *ptc_handler,
		uint8_t queue_id) {
	error_code_t error = 0;

	if (queue_id > 3) {
		error = 1;
	} else if (!ptc_handler) {
		error = 2;
	} else if (!ptc_handler->raw_tc_mem_descriptor.p_tc_bytes) {
		error = 3;
	} else if (0 == PendingActionNumber[queue_id]) {
		error = 4;
	} else {

		uint8_t head_index;
		uint16_t num_of_bytes;


		//Get Head where the ev_action is
		head_index = PendingActionHead[queue_id];

		//Get number of bytes
		num_of_bytes = PendingActionQueue[queue_id][head_index].ev_action_pack_lenght;

		//Copy action bytes
		for (uint16_t i = 0; i < num_of_bytes; i++) {

			ptc_handler->raw_tc_mem_descriptor.p_tc_bytes[i] =
					PendingActionQueue[queue_id][head_index].ev_action_pack_bytes[i];

		}
		//set number of bytes
		ptc_handler->raw_tc_mem_descriptor.tc_num_bytes = num_of_bytes;

		//Update Head
		PendingActionHead[queue_id] = (PendingActionHead[queue_id] + 1)
				% PUS_SERVICE19_EVENT_ACTION_QUEUE_DIMENSION;

		//Update Action Number after extraction
		PendingActionNumber[queue_id]--;

	}

	return error;
}

error_code_t pus_service19_extract_ev_5_1_action(tc_handler_t *ptc_handler) {
	return (pus_service19_extract_ev_action_from_queue(ptc_handler, 0));
}

error_code_t pus_service19_extract_ev_5_2_action(tc_handler_t *ptc_handler) {

	return (pus_service19_extract_ev_action_from_queue(ptc_handler, 1));

}

error_code_t pus_service19_extract_ev_5_3_action(tc_handler_t *ptc_handler) {
	return (pus_service19_extract_ev_action_from_queue(ptc_handler, 2));

}
error_code_t pus_service19_extract_ev_5_4_action(tc_handler_t *ptc_handler) {

	return (pus_service19_extract_ev_action_from_queue(ptc_handler, 3));

}

