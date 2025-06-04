/*
 * aux_pus_service6_utils.c
 *
 *  Created on: Nov 26, 2024
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

#include "../../pus_service06/include/pus_service6/aux_pus_service6_utils.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "pus_service6/aux_pus_service6_tc_exec.h"
#include "pus_service6/aux_pus_service6_utils.h"
#include "public/pus_service1.h"
#include "public/pus_service6.h"

bool_t pus_service6_is_address_valid(uint8_t MemID, uint32_t address) {

	return true;

}

bool_t pus_service6_is_MemID_WriteEnabled(uint8_t MemID) {

	return true;

}

error_code_t pus_service6_MemID_write_data(uint8_t MemID, uint8_t *pdata,
		uint32_t address, uint16_t length) {
	error_code_t error = 0;

	return error;

}

error_code_t pus_service6_MemID_read_data(uint8_t MemID, uint8_t *pdata,
		uint32_t address, uint16_t length) {

	error_code_t error = 0;

	return error;
}

error_code_t pus_service6_MemID_check_data(uint8_t MemID, uint32_t address,
		uint16_t length, uint16_t *p_checksum) {

	error_code_t error = 0;

	return error;
}

