/*
 * pus_service6.c
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


#include "../../pus_service06/include/public/pus_service6.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "pus_service6/aux_pus_service6_tc_exec.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service06/include/pus_service6/aux_pus_service6_tc_exec.h"
#include "../../pus_service06/include/pus_service6/aux_pus_service6_utils.h"


void pus_service6_exec_tc(tc_handler_t *ptc_handler) {
	switch (ptc_handler->tc_df_header.subtype) {
	case (2):
		pus_service6_exec_TC_6_2(ptc_handler);

		break;
	case (5):
		pus_service6_exec_TC_6_5(ptc_handler);
		break;
	case (9):
		pus_service6_exec_TC_6_9(ptc_handler);
		break;
	default:
		break;
	}

}
