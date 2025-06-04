/*
 * emu_sc_pus_service17.cpp
 *
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

EmuGSS_TCProgram17_1::EmuGSS_TCProgram17_1(uint32_t uniTime2YK,
		const char *brief) :
		EmuGSS_TCProgram(uniTime2YK, 17, 1, 0, brief) {

	NewProgram(this);
}

void EmuGSS_TCProgram17_1::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

}

EmuGSS_TCProgram17_1_DISABLE_1_7::EmuGSS_TCProgram17_1_DISABLE_1_7(
		uint32_t uniTime2YK, const char *brief) :
		EmuGSS_TCProgram(uniTime2YK, 17, 1, 0, brief) {

	DisableExecCompletionACK();
	NewProgram(this);
}

void EmuGSS_TCProgram17_1_DISABLE_1_7::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

}

#define TC_17_3_APPDATA_LENGTH 2

EmuGSS_TCProgram17_3::EmuGSS_TCProgram17_3(uint32_t uniTime2YK,
		const char *brief, uint16_t APID) :
		EmuGSS_TCProgram(uniTime2YK, 17, 3, TC_17_3_APPDATA_LENGTH, brief) {

	mAPID = APID;
	NewProgram(this);
}

void EmuGSS_TCProgram17_3::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

}
