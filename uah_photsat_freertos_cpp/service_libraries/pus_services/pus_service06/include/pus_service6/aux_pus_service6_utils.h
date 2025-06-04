/*
 * aux_pus_service6_utils.h
 *
 *  Created on: Nov 26, 2024
 *      Author: opolo70
 */

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE06_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE06_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "public/config.h"
#include <public/pus_tc_handler.h>

#define PUS_SERVICE6_MAX_NUM_OF_BYTES_MEM_LOAD 256
#define PUS_SERVICE6_MAX_NUM_OF_BYTES_MEM_DUMP 256
#define PUS_SERVICE6_MAX_NUM_OF_BYTES_MEM_CHECK 256

bool_t pus_service6_is_address_valid(uint8_t MemID, uint32_t address);

bool_t pus_service6_is_MemID_WriteEnabled(uint8_t MemID);

error_code_t pus_service6_MemID_write_data(uint8_t MemID, uint8_t *pdata,
		uint32_t address, uint16_t length);

error_code_t pus_service6_MemID_read_data(uint8_t MemID, uint8_t *pdata,
		uint32_t address, uint16_t length);

error_code_t pus_service6_MemID_check_data(uint8_t MemID, uint32_t address,
		uint16_t length, uint16_t *p_checksum);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE06_INCLUDE_PUS_SERVICE6_AUX_PUS_SERVICE6_UTILS_H_ */
