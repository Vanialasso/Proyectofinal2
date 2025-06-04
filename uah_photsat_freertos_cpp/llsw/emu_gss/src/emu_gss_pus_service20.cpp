/*
 * emu_sc_pus_service3.cpp
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
#include "public/pus_sys_data_pool.h"
#include "emu_gss/emu_gss_sys_data_pool.h"

#define TC_20_1_APPDATA_LENGTH 3
#define TC_20_3_UINT8_APPDATA_LENGTH 4
#define TC_20_3_UINT32_APPDATA_LENGTH 7

EmuGSS_TCProgram20_1::EmuGSS_TCProgram20_1(uint32_t uniTime2YK,
		const char *brief, uint16_t pid) :
		EmuGSS_TCProgram(uniTime2YK, 20, 1,
		TC_20_1_APPDATA_LENGTH, brief) {

	mPID = pid;
	NewProgram(this);
}

void EmuGSS_TCProgram20_1::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPID);

}

EmuGSS_TCProgram20_3_uint8::EmuGSS_TCProgram20_3_uint8(uint32_t uniTime2YK,
		const char *brief, uint16_t pid, uint8_t value) :
		EmuGSS_TCProgram(uniTime2YK, 20, 3,
		TC_20_3_UINT8_APPDATA_LENGTH, brief) {

	mPID = pid;
	mValue = value;

	NewProgram(this);
}

void EmuGSS_TCProgram20_3_uint8::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPID);
	SetNextUInt8(mValue);

}

EmuGSS_TCProgram20_3_uint32::EmuGSS_TCProgram20_3_uint32(uint32_t uniTime2YK,
		const char *brief, uint16_t pid, uint32_t value) :
		EmuGSS_TCProgram(uniTime2YK, 20, 3,
		TC_20_3_UINT32_APPDATA_LENGTH, brief) {

	mPID = pid;
	mValue = value;

	NewProgram(this);
}

void EmuGSS_TCProgram20_3_uint32::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPID);
	SetNextUInt32(mValue);

}
//********************************************************+
void EmuGSS_ShowServ20TM(const struct tm_mem_descriptor *pTMDescriptor) {

	GSSServ20TMHandler serv20TMHandler(pTMDescriptor->p_tm_bytes);
	serv20TMHandler.ShowTM();

}

GSSServ20TMHandler::GSSServ20TMHandler(const uint8_t *pTMBytes) :
		GSSTMHandler(pTMBytes) {

}

void GSSServ20TMHandler::ShowTM() {

	switch (mDFHeader.subtype) {
	case (2):
		ShowTM_20_2();
		break;

	}
}

void GSSServ20TMHandler::ShowTM_20_2() {

	//N=1 -> TM
	uint8_t N;

	N = GetNextUInt8AppDataField();

	for (int i = 0; i < N; i++) {

		uint16_t pid = GetNextUInt16AppDataField();

		data_pool_item_type_t param_type = sys_data_pool_item_type(pid);

		data_pool_item_t data_item;

		switch (param_type) {
		case (uint32_item_type): {
			data_item.uint32_data = GetNextUInt32AppDataField();


		}
			break;
		case (uint8_item_type): {
			data_item.uint8_data= GetNextUInt8AppDataField();

		}

		default:
			break;
		};
		printf(" PID %d value =",pid);

		GSSSysDataPool::ShowPIDValue(pid, data_item);
		printf("\n");
	}

}
