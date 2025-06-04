/*
 * asw_pus_service17_aux.c
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


#include <pus_service17/aux_pus_service17_exec_tc.h>
#include "public/ccsds_pus.h"
#include "public/crc.h"

#include "public/pus_service1.h"
#include "public/pus_service17.h"
#include "public/pus_tm_handler.h"




void pus_service17_tx_TM_17_2(
		tc_handler_t *ptc_handler){

	error_code_t error;
	tm_handler_t tm_handler;

	error=tm_handler_mem_alloc_and_startup(&tm_handler);

	if(0==error){

		//Close and Tx TM only if no error
		error=tm_handler_close_and_tx(&tm_handler,17,2);

		//free TM memory
		tm_handler_free_mem(&tm_handler);
	}

}
