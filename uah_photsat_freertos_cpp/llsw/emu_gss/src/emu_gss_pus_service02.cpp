/*
 * emu_sc_pus_service2.cpp
 *
 *  Created on: Jan 13, 2024
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

#include <public/pus_services_iface_v1.h>
#include <stdio.h>

#include "public/emu_gss_v1.h"

#define TC_2_1_APPDATA_LENGTH 5

EmuGSS_TCProgram2_1::EmuGSS_TCProgram2_1(uint32_t uniTime2YK, const char *brief,
		enum DeviceCommandOnOff_t devicecommand) :
		EmuGSS_TCProgram(uniTime2YK, 2, 1,
		TC_2_1_APPDATA_LENGTH, brief) {

	if (DeviceCommandOff == devicecommand)
		mDeviceOnOffCommand = 0x80000900;
	else
		mDeviceOnOffCommand = 0x8000090A;

	NewProgram(this);
}

//Constructior for event_action
EmuGSS_TCProgram2_1::EmuGSS_TCProgram2_1(

enum DeviceCommandOnOff_t devicecommand) :
		EmuGSS_TCProgram(0, 2, 1,
		TC_2_1_APPDATA_LENGTH, "NULL") {

	if (DeviceCommandOff == devicecommand)
		mDeviceOnOffCommand = 0x80000900;
	else
		mDeviceOnOffCommand = 0x8000090A;

	//No Program
	//NewProgram(this);
}

void EmuGSS_TCProgram2_1::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	//N=1
	SetNextUInt8(1);

	SetNextUInt32(mDeviceOnOffCommand);

}


EmuGSS_TCProgram2_1_BadAddress::EmuGSS_TCProgram2_1_BadAddress(uint32_t uniTime2YK, const char *brief,
		uint32_t bad_address) :
		EmuGSS_TCProgram(uniTime2YK, 2, 1,
		TC_2_1_APPDATA_LENGTH, brief) {

	mDeviceOnOffBadAddress=bad_address;

	NewProgram(this);
}

void EmuGSS_TCProgram2_1_BadAddress::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	//N=1
	SetNextUInt8(1);

	SetNextUInt32(mDeviceOnOffBadAddress);

}
