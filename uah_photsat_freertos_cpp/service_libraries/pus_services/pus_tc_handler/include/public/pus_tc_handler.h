/*
 * inta_tc_handler.h
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

#ifndef ASW_DATACLASSES_CDTCHANDLER_INCLUDE_CDTCHANDLER_TC_HANDLER_H_
#define ASW_DATACLASSES_CDTCHANDLER_INCLUDE_CDTCHANDLER_TC_HANDLER_H_

#include "public/basic_types.h"
#include "public/ccsds_pus.h"
#include "public/tmtc_dyn_mem.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Enumerate to control the task that executes the TC
 *
 */

/**
 * \brief Structure to handle a TC and enable its execution.
 *
 */
struct tc_handler {

	//!tc descriptor
	tc_mem_descriptor_t raw_tc_mem_descriptor;

	//!tc packet primary header
	ccsds_packet_primary_header_t tc_packet_header;

	//!tc data field header
	ccsds_pus_tc_df_header_t tc_df_header;

	//!tc packet error ctrl
	uint16_t tc_packet_err_ctrl;

	//! pointer to the tc app data index during execution
	uint16_t tc_exec_app_data_index;

};

//!Typedef of tc handler
typedef struct tc_handler tc_handler_t;

/**
 * \brief build the tc_handler from descriptor
 * \param ptc_handler pointer to the tc handler
 * \param tc_dcrptor tc descriptor
 * \return 0 if no error
 */
error_code_t tc_handler_build_from_descriptor(tc_handler_t *ptc_handler,
		tc_mem_descriptor_t tc_dcrptor);

/**
 * \brief return if the acceptance acknowledge is enabled
 * \param ptc_handler pointer to the tc handler
 * \return true if acceptance acknowledge is enabled
 */

bool_t tc_handler_is_tc_accept_ack_enabled(tc_handler_t *ptc_handler);

/**
 * \brief return if the start of execution acknowledge is enabled
 * \param ptc_handler pointer to the tc handler
 * \return true if the start of execution acknowledge is enabled
 */

bool_t tc_handler_is_tc_start_exec_ack_enabled(tc_handler_t *ptc_handler);

/**
 * \brief return if the progress of execution acknowledge is enabled
 * \param ptc_handler pointer to the tc handler
 * \return true if the progress of execution acknowledge is enabled
 */

bool_t tc_handler_is_tc_progress_exec_ack_enabled(
		tc_handler_t *ptc_handler);

/**
 * \brief return if the completion of execution acknowledge is enabled
 * \param ptc_handler pointer to the tc handler
 * \return true if the completion of execution acknowledge is enabled
 */
bool_t tc_handler_is_tc_completion_exec_ack_enabled(
		tc_handler_t *ptc_handler);

/**
 * \brief tc execution startup
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t tc_handler_start_up_execution(tc_handler_t *ptc_handler);

/**
 * \brief free the memory of the tc
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t tc_handler_free_memory(tc_handler_t *ptc_handler);


bool_t tc_handler_is_valid_next_appdata_field(tc_handler_t *ptc_handler,
		uint16_t field_size);

error_code_t tc_handler_get_uint8_appdata_field(tc_handler_t *ptc_handler,
		uint8_t *p_data);

error_code_t tc_handler_get_uint16_appdata_field(tc_handler_t *ptc_handler,
		uint16_t *p_data);

error_code_t tc_handler_get_uint32_appdata_field(tc_handler_t *ptc_handler,
		uint32_t *p_data);

error_code_t tc_handler_get_uint64_appdata_field(tc_handler_t *ptc_handler,
		uint64_t *p_data);

error_code_t tc_handler_get_int8_appdata_field(tc_handler_t *ptc_handler,
		int8_t *p_data);

error_code_t tc_handler_get_int16_appdata_field(tc_handler_t *ptc_handler,
		int16_t *p_data);

error_code_t tc_handler_get_int32_appdata_field(tc_handler_t *ptc_handler,
		int32_t *p_data);

error_code_t tc_handler_get_int64_appdata_field(tc_handler_t *ptc_handler,
		int64_t *p_data);

error_code_t tc_handler_get_float_appdata_field(tc_handler_t *ptc_handler,
		float *p_data);

error_code_t tc_handler_get_byte_array_appdata_field(
		tc_handler_t *ptc_handler, byte_t *p_array, uint16_t array_bytes);

error_code_t tc_handler_get_tc_mem_descriptor_from_remaining_appdata(
		tc_handler_t *ptc_handler, tc_mem_descriptor_t *ptc_mem_descriptor);

#ifdef __cplusplus
}
#endif
#endif /* ASW_DATACLASSES_CDTCHANDLER_INCLUDE_CDTCHANDLER_TC_HANDLER_H_ */
