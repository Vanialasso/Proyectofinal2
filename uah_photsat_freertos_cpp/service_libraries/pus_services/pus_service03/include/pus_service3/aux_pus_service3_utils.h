/*
 * asw_pus_service3_aux.h
 *
 *  Created on: Oct 24, 2024
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE03_INCLUDE_PUS_SERVICE3_AUX_PUS_SERVICE3_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE03_INCLUDE_PUS_SERVICE3_AUX_PUS_SERVICE3_UTILS_H_

#include "public/basic_types.h"
#include "public/config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief get the SID index in the HKConfig structure
 * \param struct_id housekeeping structure identifier
 * \param p_validindex pointer to the a valid index
 * \return 0 only if SID is valid, 1 if not
 */
error_code_t pus_service3_get_SID_index(uint16_t struct_id,
		uint8_t *p_validindex);

#ifdef __cplusplus
}
#endif
#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE03_INCLUDE_PUS_SERVICE3_AUX_PUS_SERVICE3_UTILS_H_ */
