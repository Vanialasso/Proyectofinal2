/*
 * asw_pus_sys_data_pool.h
 *
 *  Created on: Oct 22, 2024
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

#ifndef SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_PUS_SYS_DATA_POOL_INCLUDE_ASW_PUS_SYS_DATA_POOL_H_
#define SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_PUS_SYS_DATA_POOL_INCLUDE_ASW_PUS_SYS_DATA_POOL_H_

#include "public/config.h"
#include "public/basic_types.h"
#include "sys_data_pool_pids.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_DATA_POOL_UINT32_PARAMS 12
#define SYS_DATA_POOL_UINT8_PARAMS 	18
#define SYS_DATA_POOL_INT16_PARAMS 	6

#define SYS_DATA_POOL_NOT_UPDATABLE_VIA_TC_UINT32_PARAMS 	5

#define SYS_DATA_POOL_NOT_UPDATABLE_VIA_TC_UINT16_PARAMS 	6

enum data_pool_item_type {
	uint32_item_type, uint8_item_type, int16_item_type, no_valid_param_id
};

typedef enum data_pool_item_type data_pool_item_type_t;

union data_pool_item {
	uint32_t 	uint32_data;
	uint8_t 	uint8_data;
	int16_t 	int16_data;
};



typedef union data_pool_item data_pool_item_t;


/**
 * \brief return true if the system data pool param id is valid
 * \param parameter identifier
 * \return true if the system data pool param id is valid, false if not
 */
bool_t sys_data_pool_is_PID_valid(uint16_t param_id);



/**
 * \brief calculate the system data pool item type
 * \param parameter identifier
 * \return the data pool item type
 */
data_pool_item_type_t sys_data_pool_item_type(uint16_t param_id);

/**
 * \brief return the system data pool item size in bytes
 * \param type item type
 * \return the size of the item type in bytes, 0 if type is no_valid_param_id
 */
uint16_t sys_data_pool_item_type_size(data_pool_item_type_t type);

/**
 * \brief return the Parameter size in bytes
 * \param param_id Parameter identifier
 * \return the Parameter size in bytes, 0 if parameter is not valid_
 */
uint16_t sys_data_pool_PID_size(uint16_t param_id);


/**
 * \brief return true if the system data pool param id can be updated via TC
 * \param parameter identifier
 * \return true if the system data pool param an be updated via TC, false if not
 */
bool_t sys_data_pool_is_PID_update_via_TC_enabled(uint16_t param_id);



/**
 * \brief get the uint8_t parameter value whose identifier is param_id
 * \param valid_param_id parameter identifier
 * \return the uint8_t parameter value (0 if the param_id is not valid)
 */
uint8_t sys_data_pool_get_uint8_param(uint16_t valid_param_id);

/**
 * \brief get the int16_t parameter value whose identifier is param_id
 * \param valid_param_id parameter identifier
 * \return the int16_t parameter value (0 if the param_id is not valid)
 */
int16_t sys_data_pool_get_int16_param(uint16_t valid_param_id);


/**
 * \brief get the uint32_t parameter value whose identifier is param_id
 * \param valid_param_id parameter identifier
 * \return the uint32_t parameter value (0 if the param_id is not valid)
 */
uint32_t sys_data_pool_get_uint32_param(uint16_t valid_param_id);

/**
 * \brief get the data_pool_item value
 * \param param_id parameter identifier
 * \param p_item pointer where the item is copied
 * \return error, 0 if param_id is valid and p_item is not NULL
 * , 1 if p_valid_item is NULL and 2 if param_id is not valid
 */
error_code_t sys_data_pool_get_param_value(uint16_t param_id
		,data_pool_item_t *p_item);



/**
 * \brief set parameter value whose identifier is param_id from a serialized bytes
 * \param param_id parameter identifier
 * \param p_serialized_bytes pointer to serialized bytes
 * \return error, 0 if param_id is valid and p_serialized_bytes is not NULL
 * , 1 if p_serialized_bytes is NULL and 2 if param_id is not valid
 */
error_code_t sys_data_pool_set_param_from_serialized_bytes(uint16_t param_id,
		const uint8_t *p_serialized_bytes);

/**
 * \brief set item value from a serialized bytes
 * \param item_type the type of the item
 * \param p_serialized_bytes pointer to serialized bytes
 * \param p_item pointer to serialized bytes
 * \return error: 0 if item_type != no_valid_param_id, p_serialized_bytes!=NULL
 *  p_item !=NULL; 1 if p_serialized_bytes is NULL; 2 if p_item is NULL
 *  ; 3 if item_type == no_valid_param_id
 */
error_code_t sys_data_pool_set_item_from_serialized_bytes(
		data_pool_item_type_t item_type, const uint8_t *p_serialized_bytes,
		data_pool_item_t *p_item);

/**
 * \brief set uint8_t parameter value whose identifier is param_id
 * \param param_id parameter identifier
 * \return error, 0 if param_id is valid, !=0 if not
 */
error_code_t sys_data_pool_set_uint8_param(uint16_t param_id,
		uint8_t param_value);

/**
 * \brief set int16_t parameter value whose identifier is param_id
 * \param param_id parameter identifier
 * \return error, 0 if param_id is valid, !=0 if not
 */
error_code_t sys_data_pool_set_int16_param(uint16_t param_id,
		int16_t param_value);


/**
 * \brief set uint32_t parameter value whose identifier is param_id
 * \param param_id parameter identifier
 * \return error, 0 if param_id is valid, !=0 if not
 */
error_code_t sys_data_pool_set_uint32_param(uint16_t param_id,
		uint32_t param_value);


/**
 * \brief check if the system data pool item 1 is higher than item2
 *  \param type items type
 * \param p_item1 pointer to item 1 used in the comparison
 * \param p_item2 pointer to item 2 used in the comparison
 * \return the result of the comparison, (false if any pointer is not valid)
 *
 */
bool_t sys_data_pool_item1_higher_than_item2(data_pool_item_type_t type,
		const data_pool_item_t *const p_item1,
		const data_pool_item_t *const p_item2);


/**
 * \brief check if the system data pool param is higher than limit
 * \param valid_param_id valid parameter identifier
 * \param p_limit_item pointer to item used in the comparison
 * \return the result of the comparison, false if the pointer
 * or the param is not valid
 */
bool_t sys_data_pool_param_higher_than_limit(uint16_t valid_param_id,
		const data_pool_item_t *const p_limit_item);

/**
 * \brief check if the system data pool param is lower than limit
 * \param valid_param_id valid parameter identifier
 * \param p_limit_item pointer to item used in the comparison
 * \return the result of the comparison, false if the pointer
 * or the param is not valid
 */
bool_t sys_data_pool_param_lower_than_limit(uint16_t valid_param_id,
		const data_pool_item_t *const p_limit_item);

/**
 * \brief update a mean value with the param value
 * \param valid_param_id valid parameter identifier
 * \param p_mean pointer to the current mean value
 * \return 1 if the param is not valid
 */
error_code_t  sys_data_pool_param_update_mean(uint16_t valid_param_id,
		data_pool_item_t *const p_mean_value, uint32_t samples);

/**
 * \brief compare using a mask a system data pool param with an item
 * \param valid_param_id valid parameter identifier
 * \param p_item pointer to the item
 * \param p_mask_item pointer to the mask to apply
 * \return the result of the comparison, false if any pointer
 * or the param is not valid
 */
bool_t sys_data_pool_masked_param_equal_than_masked_item(
		uint16_t valid_param_id, const data_pool_item_t *const p_item,
		const data_pool_item_t *const p_mask_item);


#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_PUS_SYS_DATA_POOL_INCLUDE_ASW_PUS_SYS_DATA_POOL_H_ */
