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

#ifndef ASW_DATACLASSES_CDTMHANDLER_INCLUDE_CDTMHANDLER_TM_HANDLER_H_
#define ASW_DATACLASSES_CDTMHANDLER_INCLUDE_CDTMHANDLER_TM_HANDLER_H_

#include "public/basic_types.h"
#include "public/ccsds_pus.h"
#include "public/tmtc_dyn_mem.h"
#include "public/pus_sys_data_pool.h"

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
struct tm_handler {

	//!tm descriptor
	tm_mem_descriptor_t tm_mem_descriptor;

	//!tc packet primary header
	ccsds_packet_primary_header_t tm_packet_header;

	//!tm data field header
	ccsds_pus_tm_df_header_t tm_df_header;

	//! pointer to the tc app data index during execution
	uint16_t tm_build_app_data_index;

};

//!Typedef of tc handler
typedef struct tm_handler tm_handler_t;



/**
 * \brief tm memory allocation and startup.
 * \param ptm_handler pointer to the tm handler
 * \return 0 if no error
 */
error_code_t tm_handler_mem_alloc_and_startup(tm_handler_t *ptm_handler);

/**
 * \brief close tm .
 * \param ptm_handler pointer to the tm handler
 * param type service type
 * \param subtype service subtype
 * \return 0 if no error
 */
error_code_t tm_handler_close_and_tx(tm_handler_t *ptm_handler, uint8_t type
		, uint8_t subtype);


/**
 * \brief tm free memory.
 * \param ptm_handler pointer to the tm handler
 * \return 0 if no error
 */
error_code_t tm_handler_free_mem(tm_handler_t *ptm_handler);


/**
 * \brief is valid the next field of the tm
 * \param ptc_handler pointer to the tm handler
 *
 * \return 1 if no error
 */
bool_t tm_handler_is_valid_next_appdata_field(tm_handler_t *ptm_handler,
		uint16_t field_size);

error_code_t tm_handler_append_uint8_appdata_field(tm_handler_t *ptm_handler,
		uint8_t data);

error_code_t tm_handler_append_uint16_appdata_field(tm_handler_t *ptm_handler,
		uint16_t data);

error_code_t tm_handler_append_uint32_appdata_field(tm_handler_t *ptm_handler,
		uint32_t data);

error_code_t tm_handler_append_uint64_appdata_field(tm_handler_t *ptm_handler,
		uint64_t data);

error_code_t tm_handler_append_int8_appdata_field(tm_handler_t *ptm_handler,
		int8_t data);

error_code_t tm_handler_append_int16_appdata_field(tm_handler_t *ptm_handler,
		int16_t data);

error_code_t tm_handler_append_int32_appdata_field(tm_handler_t *ptm_handler,
		int32_t data);

error_code_t tm_handler_append_int64_appdata_field(tm_handler_t *ptm_handler,
		int64_t data);

error_code_t tm_handler_append_byte_array_appdata_field(tm_handler_t *ptm_handler
		,byte_t *p_array, uint8_t array_bytes);

error_code_t tm_handler_append_data_pool_item(tm_handler_t *ptm_handler
		,uint16_t PID,data_pool_item_t *p_data_pool_item);


#ifdef __cplusplus
}
#endif
#endif /* ASW_DATACLASSES_CDTCHANDLER_INCLUDE_CDTCHANDLER_TC_HANDLER_H_ */
