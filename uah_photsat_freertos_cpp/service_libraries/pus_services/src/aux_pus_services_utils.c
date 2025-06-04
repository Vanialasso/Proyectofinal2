/*
 * aux_pus_services_utils.c
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

#include "pus_services/aux_pus_services_utils.h"

#include "public/pus_sys_data_pool.h"

error_code_t pus_services_TM_X_Y_write_PIDValue(tm_handler_t *ptm_handler,
		uint16_t param_id) {

	error_code_t error = 0;

	data_pool_item_type_t item_type = sys_data_pool_item_type(
			param_id);

	switch (item_type) {

	case (uint32_item_type):
		error = tm_handler_append_uint32_appdata_field(ptm_handler,
				sys_data_pool_get_uint32_param(param_id));

		break;

	case (int16_item_type):
		error = tm_handler_append_int16_appdata_field(ptm_handler,
				sys_data_pool_get_int16_param(param_id));

		break;
	case (uint8_item_type):
		error = tm_handler_append_uint8_appdata_field(ptm_handler,
				sys_data_pool_get_uint8_param(param_id));

		break;
	default:
		error = 2;

	}
	return error;
}

error_code_t pus_services_TM_X_Y_write_saved_PIDValue(tm_handler_t *ptm_handler,
		uint16_t param_id,const data_pool_item_t *const p_pid_value){

	error_code_t error = 0;

	data_pool_item_type_t item_type = sys_data_pool_item_type(
			param_id);

	switch (item_type) {

	case (uint32_item_type):
		error = tm_handler_append_uint32_appdata_field(ptm_handler,
				p_pid_value->uint32_data);

		break;
	case (int16_item_type):
		error = tm_handler_append_int16_appdata_field(ptm_handler,
				p_pid_value->int16_data);

		break;

	case (uint8_item_type):
		error = tm_handler_append_uint8_appdata_field(ptm_handler,
				p_pid_value->uint8_data);

		break;
	default:
		error = 2;

	}
	return error;


}
