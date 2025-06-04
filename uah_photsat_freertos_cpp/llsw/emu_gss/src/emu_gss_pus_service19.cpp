/*
 * emu_sc_pus_service12.cpp
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

#include <public/pus_services_iface_v1.h>
#include <stdio.h>

#include "public/emu_gss_v1.h"


#define TC_2_1_APPDATA_LENGTH  5


#define TC_2_1_PACK_BYTES  (UAH_PUS_TC_APP_DATA_OFFSET + TC_2_1_APPDATA_LENGTH + 2)
#define TC_19_1_ACT_2_1_APPDATA_LENGTH (3 + TC_2_1_PACK_BYTES)

#define TC_128_1_APPDATA_LENGTH  0
#define TC_128_1_PACK_BYTES  (UAH_PUS_TC_APP_DATA_OFFSET + TC_128_1_APPDATA_LENGTH + 2)
#define TC_19_1_ACT_128_1_APPDATA_LENGTH (3 + TC_128_1_PACK_BYTES)

#define TC_129_3_APPDATA_LENGTH  0
#define TC_129_3_PACK_BYTES  (UAH_PUS_TC_APP_DATA_OFFSET + TC_129_3_APPDATA_LENGTH + 2)
#define TC_19_1_ACT_129_3_APPDATA_LENGTH (3 + TC_129_3_PACK_BYTES)


#define TC_19_2_APPDATA_LENGTH 3
#define TC_19_4_APPDATA_LENGTH 3
#define TC_19_5_APPDATA_LENGTH 3

EmuGSS_TCProgram19_1_Action_2_1::EmuGSS_TCProgram19_1_Action_2_1(
		uint32_t uniTime2YK, const char *brief, uint16_t evID,
		enum DeviceCommandOnOff_t devicecommand) :
		EmuGSS_TCProgram(uniTime2YK, 19, 1,
		TC_19_1_ACT_2_1_APPDATA_LENGTH, brief), mProgram_2_1(devicecommand) {

	mEvID = evID;
	NewProgram(this);
}

void EmuGSS_TCProgram19_1_Action_2_1::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	tc_mem_descriptor_t tc_2_1_descriptor;
	uint8_t tc_2_1_bytes[TC_2_1_PACK_BYTES];
	tc_2_1_descriptor.p_tc_bytes = tc_2_1_bytes;

	SetNextUInt8(1);
	SetNextUInt16(mEvID);

	mProgram_2_1.BuildTC(tc_2_1_descriptor);
	for (int i = 0; i < tc_2_1_descriptor.tc_num_bytes; i++)
		SetNextUInt8(tc_2_1_descriptor.p_tc_bytes[i]);

}

EmuGSS_TCProgram19_1_Action_128_1::EmuGSS_TCProgram19_1_Action_128_1(
		uint32_t uniTime2YK, const char *brief, uint16_t evID) :
		EmuGSS_TCProgram(uniTime2YK, 19, 1,
		TC_19_1_ACT_128_1_APPDATA_LENGTH, brief), mProgram_128_1() {
	mEvID = evID;
	NewProgram(this);

}

void EmuGSS_TCProgram19_1_Action_128_1::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	tc_mem_descriptor_t tc_128_1_descriptor;
	uint8_t tc_128_1_bytes[TC_128_1_PACK_BYTES];
	tc_128_1_descriptor.p_tc_bytes = tc_128_1_bytes;

	SetNextUInt8(1);
	SetNextUInt16(mEvID);

	mProgram_128_1.BuildTC(tc_128_1_descriptor);

	for (int i = 0; i < tc_128_1_descriptor.tc_num_bytes; i++)
		SetNextUInt8(tc_128_1_descriptor.p_tc_bytes[i]);

}



EmuGSS_TCProgram19_1_Action_129_3::EmuGSS_TCProgram19_1_Action_129_3(
		uint32_t uniTime2YK, const char *brief, uint16_t evID) :
		EmuGSS_TCProgram(uniTime2YK, 19, 1,
		TC_19_1_ACT_129_3_APPDATA_LENGTH, brief), mProgram_129_3() {
	mEvID = evID;
	NewProgram(this);

}

void EmuGSS_TCProgram19_1_Action_129_3::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	tc_mem_descriptor_t tc_129_3_descriptor;
	uint8_t tc_129_3_bytes[TC_129_3_PACK_BYTES];
	tc_129_3_descriptor.p_tc_bytes = tc_129_3_bytes;

	SetNextUInt8(1);
	SetNextUInt16(mEvID);

	mProgram_129_3.BuildTC(tc_129_3_descriptor);

	for (int i = 0; i < tc_129_3_descriptor.tc_num_bytes; i++)
		SetNextUInt8(tc_129_3_descriptor.p_tc_bytes[i]);

}


EmuGSS_TCProgram19_2::EmuGSS_TCProgram19_2(uint32_t uniTime2YK,
		const char *brief, uint16_t evID) :
		EmuGSS_TCProgram(uniTime2YK, 19, 2,
		TC_19_2_APPDATA_LENGTH, brief) {

	mEvID = evID;
	NewProgram(this);
}

void EmuGSS_TCProgram19_2::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {
	SetNextUInt8(1);
	SetNextUInt16(mEvID);

}

EmuGSS_TCProgram19_4::EmuGSS_TCProgram19_4(uint32_t uniTime2YK,
		const char *brief, uint16_t evID) :
		EmuGSS_TCProgram(uniTime2YK, 19, 4,
		TC_19_4_APPDATA_LENGTH, brief) {

	mEvID = evID;
	NewProgram(this);
}

void EmuGSS_TCProgram19_4::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1);
	SetNextUInt16(mEvID);

}

EmuGSS_TCProgram19_5::EmuGSS_TCProgram19_5(uint32_t uniTime2YK,
		const char *brief, uint16_t evID) :
		EmuGSS_TCProgram(uniTime2YK, 19, 5,
		TC_19_5_APPDATA_LENGTH, brief) {

	mEvID = evID;
	NewProgram(this);
}

void EmuGSS_TCProgram19_5::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1);
	SetNextUInt16(mEvID);

}

//********************************************************+
void EmuGSS_ShowServ19TM(const struct tm_mem_descriptor *pTMDescriptor) {

	GSSServ19TMHandler serv9TMHandler(pTMDescriptor->p_tm_bytes);
	serv9TMHandler.ShowTM();

}

GSSServ19TMHandler::GSSServ19TMHandler(const uint8_t *pTMBytes) :
		GSSTMHandler(pTMBytes) {

}

void GSSServ19TMHandler::ShowTM() {

}

