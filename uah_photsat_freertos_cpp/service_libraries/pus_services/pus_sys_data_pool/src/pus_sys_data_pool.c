/*
 * asw_pus_system_data_pool.c
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

#include "public/pus_sys_data_pool.h"
#include "public/serialize.h"

//From 0 to ASW_PUS_SYS_DATA_POOL_UINT32_PARAMS-1
#define SYS_DATA_POOL_FIRST_UINT32_PARAM_ID (0)
#define SYS_DATA_POOL_LAST_UINT32_PARAM_ID (SYS_DATA_POOL_UINT32_PARAMS-1)

//From ASW_PUS_SYS_DATA_POOL_UINT32_PARAMS to
//(ASW_PUS_SYS_DATA_POOL_UINT8_PARAMS + ASW_PUS_SYS_DATA_POOL_UINT8_PARAMS)
#define SYS_DATA_POOL_FIRST_UINT8_PARAM_ID (SYS_DATA_POOL_LAST_UINT32_PARAM_ID + 1)
#define SYS_DATA_POOL_LAST_UINT8_PARAM_ID (SYS_DATA_POOL_FIRST_UINT8_PARAM_ID + SYS_DATA_POOL_UINT8_PARAMS-1)

#define SYS_DATA_POOL_FIRST_INT16_PARAM_ID (SYS_DATA_POOL_LAST_UINT8_PARAM_ID +1 )
#define SYS_DATA_POOL_LAST_INT16_PARAM_ID (SYS_DATA_POOL_FIRST_INT16_PARAM_ID + SYS_DATA_POOL_INT16_PARAMS-1)



#define SYS_DATA_POOL_LAST_VALID_PARAM_ID SYS_DATA_POOL_LAST_INT16_PARAM_ID

uint32_t UInt32SystemDataPool[SYS_DATA_POOL_UINT32_PARAMS];

uint8_t UInt8SystemDataPool[SYS_DATA_POOL_UINT8_PARAMS];

int16_t Int16SystemDataPool[SYS_DATA_POOL_INT16_PARAMS];

#define UINT32Param(param_id) (UInt32SystemDataPool[param_id\
				- SYS_DATA_POOL_FIRST_UINT32_PARAM_ID])

#define UINT8Param(param_id) (UInt8SystemDataPool[param_id\
				- SYS_DATA_POOL_FIRST_UINT8_PARAM_ID])

#define INT16Param(param_id) (Int16SystemDataPool[param_id\
				- SYS_DATA_POOL_FIRST_INT16_PARAM_ID])


bool_t sys_data_pool_is_PID_valid(uint16_t param_id) {

	return (param_id <= SYS_DATA_POOL_LAST_VALID_PARAM_ID);

}

data_pool_item_type_t sys_data_pool_item_type(uint16_t param_id) {

	data_pool_item_type_t param_item_type;

	if (param_id <= SYS_DATA_POOL_LAST_UINT32_PARAM_ID) {

		param_item_type = uint32_item_type;

	} else if (param_id <= SYS_DATA_POOL_LAST_UINT8_PARAM_ID) {

		param_item_type = uint8_item_type;
	} else if (param_id <= SYS_DATA_POOL_LAST_INT16_PARAM_ID) {

		param_item_type = int16_item_type;
	} else
		param_item_type = no_valid_param_id;

	return param_item_type;

}

uint16_t sys_data_pool_item_type_size(data_pool_item_type_t type) {

	uint16_t size = 0;

	switch (type) {

	case (uint32_item_type):
		size = 4;
		break;

	case (uint8_item_type):
		size = 1;
		break;

	case (int16_item_type):
		size = 1;
		break;
	case (no_valid_param_id):
		//If not valid PID, size 0
		size = 0;
		break;
	}
	return size;

}

uint16_t sys_data_pool_PID_size(uint16_t param_id) {

	data_pool_item_type_t type;
	type = sys_data_pool_item_type(param_id);
	return (sys_data_pool_item_type_size(type));

}

bool_t sys_data_pool_is_PID_update_via_TC_enabled(uint16_t param_id) {

	//Param from 0 to NOT_UPDATABLE_VIA_TC cannot be updated by TC

	data_pool_item_type_t type = sys_data_pool_item_type(param_id);

	bool_t is_updatable;

	switch (type) {

	case (uint32_item_type):
		is_updatable = (param_id
				>= SYS_DATA_POOL_NOT_UPDATABLE_VIA_TC_UINT32_PARAMS);
		break;

	case (int16_item_type):
		is_updatable = false;
		break;

	default:

		is_updatable = true;

	}

	return is_updatable;

}

int16_t sys_data_pool_get_int16_param(uint16_t valid_param_id) {

	int16_t param_value = 0;

	if (int16_item_type == sys_data_pool_item_type(valid_param_id)) {
		param_value = INT16Param(valid_param_id);
	}
	return param_value;

}

uint8_t sys_data_pool_get_uint8_param(uint16_t valid_param_id) {

	uint8_t param_value = 0;

	if (uint8_item_type == sys_data_pool_item_type(valid_param_id)) {
		param_value = UINT8Param(valid_param_id);
	}
	return param_value;

}

uint32_t sys_data_pool_get_uint32_param(uint16_t valid_param_id) {

	uint32_t param_value = 0;

	if (uint32_item_type == sys_data_pool_item_type(valid_param_id)) {
		param_value = UINT32Param(valid_param_id);
	}
	return param_value;
}

error_code_t sys_data_pool_get_param_value(uint16_t param_id,
		data_pool_item_t *p_valid_item) {
	error_code_t error;

	if (!p_valid_item) {
		error = 1;

	} else {

		data_pool_item_type_t type;

		//Get Data Pool Item Type
		type = sys_data_pool_item_type(param_id);

		switch (type) {

		case (uint32_item_type):
			p_valid_item->uint32_data = UINT32Param(param_id);
			break;

		case (uint8_item_type):
			p_valid_item->uint8_data = UINT8Param(param_id);
			break;

		case (int16_item_type):
			p_valid_item->int16_data = INT16Param(param_id);
			break;

		case (no_valid_param_id):
			//If not valid PID, do
			error = 2;
			break;

		}

	}
	return error;

}

error_code_t sys_data_pool_set_param_from_serialized_bytes(uint16_t param_id,
		const uint8_t *p_serialized_bytes) {

	error_code_t error = 0;

	if (!p_serialized_bytes) {
		error = 1;

	} else {

		data_pool_item_type_t type;

		//Get Data Pool Item Type
		type = sys_data_pool_item_type(param_id);

		switch (type) {

		case (uint32_item_type):
			UINT32Param(param_id) = deserialize_uint32(p_serialized_bytes);
			break;

		case (uint8_item_type):
			UINT8Param(param_id) = *p_serialized_bytes;
			break;

		case (int16_item_type):
			INT16Param(param_id) = deserialize_int16(p_serialized_bytes);;
			break;

		case (no_valid_param_id):
			//If not valid PID, do
			error = 2;
			break;

		}

	}
	return error;

}

error_code_t sys_data_pool_set_item_from_serialized_bytes(
		data_pool_item_type_t item_type, const uint8_t *p_serialized_bytes,
		data_pool_item_t *p_item) {

	error_code_t error;

	if (!p_serialized_bytes) {
		error = 1;

	} else if (!p_item) {

		error = 2;
	} else {

		switch (item_type) {
		case (uint32_item_type):

			p_item->uint32_data = deserialize_uint32(p_serialized_bytes);
			break;

		case (uint8_item_type):
			p_item->uint8_data = *p_serialized_bytes;
			break;
		case (int16_item_type):
			p_item->int16_data = deserialize_int16(p_serialized_bytes);;
			break;

			break;
		case (no_valid_param_id):
			//If not valid PID, do
			error = 3;
			break;

		}

	}
	return error;
}

error_code_t sys_data_pool_set_uint8_param(uint16_t param_id,
		uint8_t param_value) {

	error_code_t error = 1;
	if (uint8_item_type == sys_data_pool_item_type(param_id)) {

		UINT8Param(param_id) = param_value;
		error = 0;
	}
	return error;
}

error_code_t sys_data_pool_set_int16_param(uint16_t param_id,
		int16_t param_value) {

	error_code_t error = 1;
	if (int16_item_type == sys_data_pool_item_type(param_id)) {

		INT16Param(param_id) = param_value;
		error = 0;
	}
	return error;
}

error_code_t sys_data_pool_set_uint32_param(uint16_t param_id,
		uint32_t param_value) {

	error_code_t error = 1;
	if (uint32_item_type == sys_data_pool_item_type(param_id)) {

		UINT32Param(param_id) = param_value;
		error = 0;
	}
	return error;
}

bool_t sys_data_pool_item1_higher_than_item2(data_pool_item_type_t type,
		const data_pool_item_t *const p_item1,
		const data_pool_item_t *const p_item2) {

	bool_t result = false;

	if (p_item1 && p_item2) {

		//Get Data Pool Item Type

		switch (type) {

		case (uint32_item_type):
			result = p_item1->uint32_data > p_item2->uint32_data;

			break;

		case (int16_item_type):
			result = p_item1->int16_data > p_item2->int16_data;
			break;
		case (uint8_item_type):
			result = p_item1->uint8_data > p_item2->uint8_data;

			break;
		case (no_valid_param_id):
			//If not valid type, result will be false

			break;

		}

	}

	return result;

}

bool_t sys_data_pool_param_higher_than_limit(uint16_t valid_param_id,
		const data_pool_item_t *const p_limit_item) {

	bool_t result = false;

	if (p_limit_item) {

		//Get Data Pool Item Type
		data_pool_item_type_t type = sys_data_pool_item_type(valid_param_id);

		switch (type) {

		case (uint32_item_type):
			result = (UINT32Param(valid_param_id) > p_limit_item->uint32_data);

			break;

		case (int16_item_type):
			result = (INT16Param(valid_param_id) > p_limit_item->int16_data);

			break;

		case (uint8_item_type):
			result = (UINT8Param(valid_param_id) > p_limit_item->uint8_data);

			break;
		case (no_valid_param_id):
			//If not valid PID, do

			break;

		}

	}

	return result;
}

bool_t sys_data_pool_param_lower_than_limit(uint16_t valid_param_id,
		const data_pool_item_t *const p_limit_item) {

	bool_t result = false;

	if (p_limit_item) {

		//Get Data Pool Item Type
		data_pool_item_type_t type = sys_data_pool_item_type(valid_param_id);

		switch (type) {

		case (uint32_item_type):
			result = (UINT32Param(valid_param_id) < p_limit_item->uint32_data);

			break;

		case (int16_item_type):
			result = (INT16Param(valid_param_id) < p_limit_item->int16_data);

			break;
		case (uint8_item_type):
			result = (UINT8Param(valid_param_id) < p_limit_item->uint8_data);

			break;
		case (no_valid_param_id):
			//If not valid PID, do

			break;

		}

	}

	return result;

}

error_code_t sys_data_pool_param_update_mean(uint16_t valid_param_id,
		data_pool_item_t *const p_mean_value, uint32_t samples) {

	error_code_t error = 0;

	if (p_mean_value) {

		//Get Data Pool Item Type
		data_pool_item_type_t type = sys_data_pool_item_type(valid_param_id);

		switch (type) {

		case (uint32_item_type): {
			uint64_t aux_mean;
			aux_mean = p_mean_value->uint32_data * samples;
			aux_mean += UINT32Param(valid_param_id);
			p_mean_value->uint32_data = aux_mean / (samples + 1);
		}
			break;

		case (int16_item_type): {
			uint16_t aux_mean;
			aux_mean = p_mean_value->int16_data * samples;
			aux_mean += INT16Param(valid_param_id);
			p_mean_value->int16_data = aux_mean / (samples + 1);
		}

		case (uint8_item_type): {
			uint16_t aux_mean;
			aux_mean = p_mean_value->uint8_data * samples;
			aux_mean += UINT8Param(valid_param_id);
			p_mean_value->uint8_data = aux_mean / (samples + 1);
		}

			break;
		case (no_valid_param_id):
			//If not valid PID, do
			error = 2;
			break;

		}

	} else {
		error = 1;
	}

	return error;

}

bool_t sys_data_pool_masked_param_equal_than_masked_item(
		uint16_t valid_param_id, const data_pool_item_t *const p_item,
		const data_pool_item_t *const p_mask_item) {

	bool_t result = false; //false is the default value

	if (p_item && p_mask_item) {

		//Get Data Pool Item Type
		data_pool_item_type_t type = sys_data_pool_item_type(valid_param_id);

		switch (type) {

		case (uint32_item_type): {

			uint32_t op1 = UINT32Param(valid_param_id)
					& p_mask_item->uint32_data;

			uint32_t op2 = p_item->uint32_data & p_mask_item->uint32_data;

			result = (op1 == op2);
		}
			break;

		case (int16_item_type): {

			int16_t op1 = INT16Param(valid_param_id) & p_mask_item->int16_data;

			int16_t op2 = p_item->int16_data & p_mask_item->int16_data;

			result = (op1 == op2);
		}
			break;

		case (uint8_item_type): {

			uint8_t op1 = UINT8Param(valid_param_id) & p_mask_item->uint8_data;

			uint8_t op2 = p_item->uint8_data & p_mask_item->uint8_data;

			result = (op1 == op2);
		}
			break;
		case (no_valid_param_id):
			//param must be valid

			break;

		}

	}

	return result;

}
