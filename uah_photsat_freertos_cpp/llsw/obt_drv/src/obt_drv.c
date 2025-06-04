/*
 * obt_drv.c
 *
 *  Created on: Oct 16, 2024
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



#include "public/emu_hw_timecode_drv_v1.h"

uint32_t obt_drv_get_current_obt(){

		return EmuHwTimeCodeGetCurrentOBT();

}

void obt_drv_set_current_obt(uint32_t next_obt){


	EmuHwTimeCodeSetNextOBT(next_obt);

}

uint16_t obt_drv_get_current_finetime_16bits(){

		return 0;
}
