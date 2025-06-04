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

#include "public/pus_tc_handler.h"
#include "public/serialize.h"

error_code_t tc_handler_build_from_descriptor(tc_handler_t *ptc_handler,
		tc_mem_descriptor_t tc_dcrptor) {
	error_code_t error = 0;

	if (!ptc_handler) {
		error = 1; //
	} else {

		//copy tc_descriptor
		ptc_handler->raw_tc_mem_descriptor = tc_dcrptor;

		//Get Headers and pack error control from tc bytes
		ccsds_pus_tc_read_fields(tc_dcrptor.p_tc_bytes,
				&ptc_handler->tc_packet_header, &ptc_handler->tc_df_header,
				&ptc_handler->tc_packet_err_ctrl);

	}
	return error;
}

#define ACCEPT_ACK_ENABLED_MASK 0x01
#define EXEC_START_ACK_ENABLED_MASK 0x02
#define EXEC_PROGRESS_ACK_ENABLED_MASK 0x04
#define EXEC_COMPLETION_ACK_ENABLED_MASK 0x08

bool_t tc_handler_is_tc_accept_ack_enabled(tc_handler_t *ptc_handler) {

	bool_t ack_enabled = false; //false if ptc_handler is not valid
	if (ptc_handler) {

		ack_enabled = ptc_handler->tc_df_header.version_ackflags
				& ACCEPT_ACK_ENABLED_MASK;
	}

	return ack_enabled;
}

bool_t tc_handler_is_tc_start_exec_ack_enabled(tc_handler_t *ptc_handler) {

	bool_t ack_enabled = false; //false if ptc_handler is not valid
	if (ptc_handler) {

		ack_enabled = ptc_handler->tc_df_header.version_ackflags
				& EXEC_START_ACK_ENABLED_MASK;
	}

	return ack_enabled;
}

bool_t tc_handler_is_tc_progress_exec_ack_enabled(tc_handler_t *ptc_handler) {

	bool_t ack_enabled = false; //false if ptc_handler is not valid
	if (ptc_handler) {

		ack_enabled = ptc_handler->tc_df_header.version_ackflags
				& EXEC_PROGRESS_ACK_ENABLED_MASK;
	}

	return ack_enabled;
}

bool_t tc_handler_is_tc_completion_exec_ack_enabled(tc_handler_t *ptc_handler) {
	bool_t ack_enabled = false; //false if ptc_handler is not valid
	if (ptc_handler) {

		ack_enabled = ptc_handler->tc_df_header.version_ackflags
				& EXEC_COMPLETION_ACK_ENABLED_MASK;
	}

	return ack_enabled;
}

error_code_t tc_handler_start_up_execution(tc_handler_t *ptc_handler) {

	error_code_t error = 0;

	if (!ptc_handler) {
		error = 1; //no handler in pointer
	} else if (!ptc_handler->raw_tc_mem_descriptor.p_tc_bytes) {

		error = 2; //no memory in tc handler descriptor
		ptc_handler->tc_exec_app_data_index = UAH_PUS_TC_MAX_NUM_OF_BYTES;

	} else {

		ptc_handler->tc_exec_app_data_index = UAH_PUS_TC_APP_DATA_OFFSET;

	}

	return error;

}

error_code_t tc_handler_free_memory(tc_handler_t *ptc_handler) {

	error_code_t error = 0;

	if (!ptc_handler) {
		error = 1; //no handler in pointer
	} else if (!ptc_handler->raw_tc_mem_descriptor.p_tc_bytes) {

		error = 2; //no memory in tc handler descriptor

	} else {

		tmtc_pool_free(ptc_handler->raw_tc_mem_descriptor.p_tc_bytes);
	}

	ptc_handler->raw_tc_mem_descriptor.tc_num_bytes = 0;

	return error;

}

bool_t tc_handler_is_valid_next_appdata_field(tc_handler_t *ptc_handler,
		uint16_t field_size) {

	bool_t is_valid = false;

	if (ptc_handler) {

		//required tc size
		uint32_t required_tc_size = ptc_handler->tc_exec_app_data_index
				+ field_size;

		//tc size
		uint32_t tc_num_bytes = ptc_handler->raw_tc_mem_descriptor.tc_num_bytes;

		//valid only if tc size >= required size
		is_valid = (required_tc_size <= tc_num_bytes);
	}

	return is_valid;

}

error_code_t tc_handler_get_uint8_appdata_field(tc_handler_t *ptc_handler,
		uint8_t *p_data) {

	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 1)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = ptc_handler->raw_tc_mem_descriptor.p_tc_bytes[index];

		ptc_handler->tc_exec_app_data_index++;
	} else {
		error = 1;
	}

	return error;
}

error_code_t tc_handler_get_uint16_appdata_field(tc_handler_t *ptc_handler,
		uint16_t *p_data) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 2)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = deserialize_uint16(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index += 2;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tc_handler_get_uint32_appdata_field(tc_handler_t *ptc_handler,
		uint32_t *p_data) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 4)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = deserialize_uint32(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index += 4;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tc_handler_get_uint64_appdata_field(tc_handler_t *ptc_handler,
		uint64_t *p_data) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 8)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = deserialize_uint64(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index += 8;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tc_handler_get_int8_appdata_field(tc_handler_t *ptc_handler,
		int8_t *p_data) {

	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 1)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data =
				*(int8_t*) (ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index++;
	} else {
		error = 1;
	}

	return error;
}

error_code_t tc_handler_get_int16_appdata_field(tc_handler_t *ptc_handler,
		int16_t *p_data) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 2)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = deserialize_int16(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index += 2;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tc_handler_get_int32_appdata_field(tc_handler_t *ptc_handler,
		int32_t *p_data) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 4)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = deserialize_int32(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index += 4;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tc_handler_get_int64_appdata_field(tc_handler_t *ptc_handler,
		int64_t *p_data) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, 8)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		*p_data = deserialize_int64(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

		ptc_handler->tc_exec_app_data_index += 8;
	} else {
		error = 1;
	}

	return error;

}

error_code_t tc_handler_get_float_appdata_field(tc_handler_t *ptc_handler,
		float *p_data){

		error_code_t error = 0;
		if (tc_handler_is_valid_next_appdata_field(ptc_handler, 4)) {
			uint16_t index = ptc_handler->tc_exec_app_data_index;

			*p_data = deserialize_float(
					ptc_handler->raw_tc_mem_descriptor.p_tc_bytes + index);

			ptc_handler->tc_exec_app_data_index += 4;
		} else {
			error = 1;
		}

		return error;

}

error_code_t tc_handler_get_byte_array_appdata_field(tc_handler_t *ptc_handler,
		byte_t *p_array, uint16_t array_bytes) {
	error_code_t error = 0;
	if (tc_handler_is_valid_next_appdata_field(ptc_handler, array_bytes)) {
		uint16_t index = ptc_handler->tc_exec_app_data_index;

		for (uint16_t i = 0; i < array_bytes; i++) {

			*(p_array + i) =
					*(byte_t*) (ptc_handler->raw_tc_mem_descriptor.p_tc_bytes
							+ index + i);
		}

		ptc_handler->tc_exec_app_data_index += array_bytes;

	} else {
		error = 1;
	}

	return error;

}

#define CRC_BYTES 2

error_code_t tc_handler_get_tc_mem_descriptor_from_remaining_appdata(
		tc_handler_t *ptc_handler, tc_mem_descriptor_t *ptc_mem_descriptor){

	error_code_t error = 0;

	uint16_t remaining_bytes = ptc_handler->raw_tc_mem_descriptor.tc_num_bytes
			- CRC_BYTES
			- ptc_handler->tc_exec_app_data_index;

	if (remaining_bytes) {

		ptc_mem_descriptor->p_tc_bytes=
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes
				+ ptc_handler->tc_exec_app_data_index;

		ptc_mem_descriptor->tc_num_bytes=remaining_bytes;

	} else {

		ptc_mem_descriptor->p_tc_bytes=NULL;
		ptc_mem_descriptor->tc_num_bytes=0;

		error = 1;
	}

	return error;
}

