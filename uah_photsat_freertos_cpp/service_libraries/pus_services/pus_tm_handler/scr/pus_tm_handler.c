/*
 * tc_handler.c
 *
 *  Created on: Oct 7, 2024
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

#include <public/pus_tm_handler.h>
#include "public/serialize.h"
#include "public/sc_channel_drv_v1.h"
#include "public/obt_drv.h"

uint16_t packet_seq_ctrl = 0;

uint16_t get_next_tm_packet_seq_ctrl() {

	uint16_t current_pack_seq_ctrl = packet_seq_ctrl;

	//update packet_seq_ctrl
	packet_seq_ctrl++;
	if (packet_seq_ctrl == 0x3FFF)
		packet_seq_ctrl = 0;

	return current_pack_seq_ctrl;
}

error_code_t tm_handler_mem_alloc_and_startup(tm_handler_t *ptm_handler) {

	error_code_t error = 0;

	if (ptm_handler) {
		ptm_handler->tm_mem_descriptor.p_tm_bytes = tmtc_pool_alloc();

		if (ptm_handler->tm_mem_descriptor.p_tm_bytes) {

			ptm_handler->tm_mem_descriptor.tm_num_bytes =
					UAH_PUS_TM_MAX_NUM_OF_BYTES;
			//This disable the writting in the AppData bytes
			ptm_handler->tm_build_app_data_index = UAH_PUS_TM_APP_DATA_OFFSET;

		} else {
			error = 2; //No memory available
			ptm_handler->tm_mem_descriptor.tm_num_bytes = 0;
			//This disable the writting in the AppData bytes
			ptm_handler->tm_build_app_data_index = UAH_PUS_TM_MAX_NUM_OF_BYTES;

		}
	} else { //No valid ptm_handler;

		error = 1;
	}
	return error;
}

void tm_handler_build_packet_header(tm_handler_t *pvalidtm_handler) {

	pvalidtm_handler->tm_packet_header.packet_id = ccsds_pus_tm_build_packet_id(
			UAH_APID);

	pvalidtm_handler->tm_packet_header.packet_seq_ctrl =
			ccsds_pus_tmtc_get_seq_count(get_next_tm_packet_seq_ctrl());

	pvalidtm_handler->tm_packet_header.packet_length =
			pvalidtm_handler->tm_build_app_data_index
					- UAH_PUS_TM_TC_PACKET_HEADER_SIZE - 1;

}

void tm_handler_serialize_packet_header(tm_handler_t *pvalidtm_handler) {

	uint8_t *p_packet_header = pvalidtm_handler->tm_mem_descriptor.p_tm_bytes;

	serialize_uint16(pvalidtm_handler->tm_packet_header.packet_id,
			&p_packet_header[0]);

	serialize_uint16(pvalidtm_handler->tm_packet_header.packet_seq_ctrl,
			&p_packet_header[2]);

	serialize_uint16(pvalidtm_handler->tm_packet_header.packet_length,
			&p_packet_header[4]);

}

void tm_handler_build_df_header(tm_handler_t *pvalidtm_handler, uint8_t type,
		uint8_t subtype) {

	pvalidtm_handler->tm_df_header.version_obtstatus =
	UAH_PUS_VERSION_TIMESTATUS;

	pvalidtm_handler->tm_df_header.type = type;
	pvalidtm_handler->tm_df_header.subtype = subtype;

	//Use only of type counter is provided
	pvalidtm_handler->tm_df_header.msgtype_counter = 0;

	pvalidtm_handler->tm_df_header.destinationID = UAH_DESTINATION_ID;

	pvalidtm_handler->tm_df_header.obt_secs = obt_drv_get_current_obt();

	//
	pvalidtm_handler->tm_df_header.obt_finetime =
			obt_drv_get_current_finetime_16bits();

}

void tm_handler_serialize_df_header(tm_handler_t *pvalidtm_handler) {

	uint8_t *p_df_header = pvalidtm_handler->tm_mem_descriptor.p_tm_bytes +
	UAH_PUS_TM_TC_DF_HEADER_OFFSET;

	p_df_header[0] = pvalidtm_handler->tm_df_header.version_obtstatus;
	p_df_header[1] = pvalidtm_handler->tm_df_header.type;
	p_df_header[2] = pvalidtm_handler->tm_df_header.subtype;

	//Use only of type counter is provided
	serialize_uint16(pvalidtm_handler->tm_df_header.msgtype_counter,
			&p_df_header[3]);

	serialize_uint16(pvalidtm_handler->tm_df_header.destinationID,
			&p_df_header[5]);

	//OBT
	serialize_uint32(pvalidtm_handler->tm_df_header.obt_secs, &p_df_header[7]);

	//FineTime
	serialize_uint16(pvalidtm_handler->tm_df_header.obt_finetime,
			&p_df_header[9]);

}

error_code_t tm_handler_close_and_tx(tm_handler_t *ptm_handler, uint8_t type,
		uint8_t subtype) {

	error_code_t error = 0;

	if (ptm_handler) {

		if (ptm_handler->tm_mem_descriptor.p_tm_bytes) {

			tm_handler_build_packet_header(ptm_handler);
			tm_handler_build_df_header(ptm_handler, type, subtype);

			tm_handler_serialize_packet_header(ptm_handler);
			tm_handler_serialize_df_header(ptm_handler);

			//Tx Telemetry
			SC_Channel_TxTM(&ptm_handler->tm_mem_descriptor);

		} else {
			//No memory, set num_bytes to 0
			ptm_handler->tm_mem_descriptor.tm_num_bytes = 0;
			error = 2;
		}

	} else { //No valid ptm_handler;

		error = 1;
	}
	return error;
}

error_code_t tm_handler_free_mem(tm_handler_t *ptm_handler) {

	error_code_t error = 0;

	if (ptm_handler) {

		if (ptm_handler->tm_mem_descriptor.p_tm_bytes) {

			tmtc_pool_free(ptm_handler->tm_mem_descriptor.p_tm_bytes);

		} else {
			error = 2;
		}
		ptm_handler->tm_mem_descriptor.tm_num_bytes = 0;
	} else {

		error = 1;
	}
	return error;

}

bool_t tm_handler_is_valid_next_appdata_field(tm_handler_t *ptm_handler,
		uint16_t field_size) {

	bool_t is_valid = false;

	uint32_t required_tm_size = ptm_handler->tm_build_app_data_index
			+ field_size;
	uint32_t tm_num_bytes = ptm_handler->tm_mem_descriptor.tm_num_bytes;
	if (ptm_handler) {
		is_valid = (required_tm_size <= tm_num_bytes);
	}

	return is_valid;

}

error_code_t tm_handler_append_uint8_appdata_field(tm_handler_t *ptm_handler,
		uint8_t data) {

	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 1)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		ptm_handler->tm_mem_descriptor.p_tm_bytes[index] = data;

		ptm_handler->tm_build_app_data_index++;
	} else {
		error = 1;
	}

	return error;
}

error_code_t tm_handler_append_uint16_appdata_field(tm_handler_t *ptm_handler,
		uint16_t data) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 2)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		serialize_uint16(data,
				ptm_handler->tm_mem_descriptor.p_tm_bytes + index);

		ptm_handler->tm_build_app_data_index += 2;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_uint32_appdata_field(tm_handler_t *ptm_handler,
		uint32_t data) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 4)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		serialize_uint32(data,
				ptm_handler->tm_mem_descriptor.p_tm_bytes + index);

		ptm_handler->tm_build_app_data_index += 4;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_uint64_appdata_field(tm_handler_t *ptm_handler,
		uint64_t data) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 8)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		serialize_uint64(data,
				ptm_handler->tm_mem_descriptor.p_tm_bytes + index);

		ptm_handler->tm_build_app_data_index += 8;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_int8_appdata_field(tm_handler_t *ptm_handler,
		int8_t data) {

	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 1)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		ptm_handler->tm_mem_descriptor.p_tm_bytes[index] = *(uint8_t*) &data;

		ptm_handler->tm_build_app_data_index++;
	} else {
		error = 1;
	}

	return error;
}

error_code_t tm_handler_append_int16_appdata_field(tm_handler_t *ptm_handler,
		int16_t data) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 2)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		serialize_int16(data,
				ptm_handler->tm_mem_descriptor.p_tm_bytes + index);

		ptm_handler->tm_build_app_data_index += 2;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_int32_appdata_field(tm_handler_t *ptm_handler,
		int32_t data) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 4)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		serialize_int32(data,
				ptm_handler->tm_mem_descriptor.p_tm_bytes + index);

		ptm_handler->tm_build_app_data_index += 4;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_int64_appdata_field(tm_handler_t *ptm_handler,
		int64_t data) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, 8)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		serialize_int64(data,
				ptm_handler->tm_mem_descriptor.p_tm_bytes + index);

		ptm_handler->tm_build_app_data_index += 8;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_byte_array_appdata_field(
		tm_handler_t *ptm_handler, byte_t *p_array, uint8_t array_bytes) {
	error_code_t error = 0;
	if (tm_handler_is_valid_next_appdata_field(ptm_handler, array_bytes)) {
		uint16_t index = ptm_handler->tm_build_app_data_index;

		for (uint8_t i = 0; i < array_bytes; i++) {

			ptm_handler->tm_mem_descriptor.p_tm_bytes[index + i] =
					*(uint8_t*) (p_array + i);
		}
		ptm_handler->tm_build_app_data_index += array_bytes;

	} else {
		error = 1;
	}

	return error;

}

error_code_t tm_handler_append_data_pool_item(tm_handler_t *ptm_handler,
		uint16_t PID, data_pool_item_t *p_data_pool_item) {

	error_code_t error;
	data_pool_item_type_t type;

	type = sys_data_pool_item_type(PID);

	switch (type) {

	case (uint32_item_type):
		error = tm_handler_append_uint32_appdata_field(ptm_handler,
				p_data_pool_item->uint32_data);
		break;
	case (uint8_item_type):
		error = tm_handler_append_uint8_appdata_field(ptm_handler,
				p_data_pool_item->uint8_data);
		break;
	case (no_valid_param_id):
		error = 1;

	}
	return error;

}
