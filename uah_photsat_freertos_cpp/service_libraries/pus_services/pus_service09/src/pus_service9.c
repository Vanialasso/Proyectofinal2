/*
 * pus_service20.c
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

#include <public/pus_service20.h>
#include <public/pus_sys_data_pool.h>
#include "public/adc_drv.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include "pus_service9/aux_pus_service9_exec_tc.h"
#include "public/pus_service1.h"
#include "public/pus_service3.h"


void pus_service9_exec_tc(tc_handler_t *ptc_handler){

	switch (ptc_handler->tc_df_header.subtype) {
	case (129):
		pus_service9_exec_TC_9_129(ptc_handler);
		break;
	default:
		break;
	}



}

