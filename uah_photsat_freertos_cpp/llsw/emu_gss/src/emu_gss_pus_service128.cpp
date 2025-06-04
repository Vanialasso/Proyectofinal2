/*
 * emu_sc_pus_service17.cpp
 *
 *  Created on: Apr 27, 2024
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



#include "public/emu_gss_v1.h"

#define TC_128_1_APPDATA_LENGTH 0

EmuGSS_TCProgram128_1::EmuGSS_TCProgram128_1(uint32_t uniTime2YK,
		const char * brief)
                        :EmuGSS_TCProgram(uniTime2YK,128,1,
                        		TC_128_1_APPDATA_LENGTH,brief){

     NewProgram(this);
}

//Constructor for event-action
EmuGSS_TCProgram128_1::EmuGSS_TCProgram128_1()
                        :EmuGSS_TCProgram(0,128,1,TC_128_1_APPDATA_LENGTH,"NULL"){

	//No Program
    // NewProgram(this);
}

void EmuGSS_TCProgram128_1::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor){


}


