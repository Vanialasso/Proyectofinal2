/*
 * asw_pus_service3_aux.c
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


#include "../../pus_service03/include/pus_service3/aux_pus_service3_utils.h"

#include "../../pus_service03/include/public/pus_service3.h"

extern HK_config_t HKConfig[PUS_SERVICE3_MAX_NUM_OF_SIDS];


error_code_t pus_service3_get_SID_index(uint16_t struct_id, uint8_t *p_validindex){

	error_code_t error= 1;

	for (int i = 0; ((i < PUS_SERVICE3_MAX_NUM_OF_SIDS) && error); i++) {
		if ((struct_id == HKConfig[i].SID) &&
				(HKConfig[i].status!=SIDConfigUnused)){

			*p_validindex= i;
			error = 0; //No error. Index founded

		}
	}
	return error;
}


