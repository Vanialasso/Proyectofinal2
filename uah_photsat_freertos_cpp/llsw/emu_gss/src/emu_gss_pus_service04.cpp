/*
 * emu_gss_pus_service4.cpp
 *
 *  Created on: Nov 21, 2024
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

#define TC_4_6_APPDATA_LENGTH 3
#define TC_4_7_APPDATA_LENGTH 3

EmuGSS_TCProgram4_1::EmuGSS_TCProgram4_1(uint32_t uniTime2YK, const char *brief) :
		EmuGSS_TCProgram(uniTime2YK, 4, 1, 0, brief) {

	NewProgram(this);
}

void EmuGSS_TCProgram4_1::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

}

EmuGSS_TCProgram4_6::EmuGSS_TCProgram4_6(uint32_t uniTime2YK, const char *brief,
		uint16_t pid) :
		EmuGSS_TCProgram(uniTime2YK, 4, 6,
		TC_4_6_APPDATA_LENGTH, brief) {

	mPID = pid;
	NewProgram(this);
}

void EmuGSS_TCProgram4_6::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPID);

}

EmuGSS_TCProgram4_7::EmuGSS_TCProgram4_7(uint32_t uniTime2YK, const char *brief,
		uint16_t pid) :
		EmuGSS_TCProgram(uniTime2YK, 4, 7,
		TC_4_7_APPDATA_LENGTH, brief) {

	mPID = pid;
	NewProgram(this);
}

void EmuGSS_TCProgram4_7::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPID);

}

//********************************************************+
void EmuGSS_ShowServ4TM(const struct tm_mem_descriptor *pTMDescriptor) {

	GSSServ4TMHandler serv4TMHandler(pTMDescriptor->p_tm_bytes);
	serv4TMHandler.ShowTM();

}

GSSServ4TMHandler::GSSServ4TMHandler(const uint8_t *pTMBytes) :
		GSSTMHandler(pTMBytes) {

}

void GSSServ4TMHandler::ShowTM() {

	switch (mDFHeader.subtype) {
	case (2):
		ShowTM_4_2();
		break;

	}
}

void GSSServ4TMHandler::ShowTM_4_2() {

	uint32_t start_time = GetNextUInt32AppDataField();
	uint32_t end_time = GetNextUInt32AppDataField();

	printf(" start_time=%d,", start_time);
	printf(" end_time=%d,", end_time);

	uint8_t N = GetNextUInt8AppDataField();

	printf(" N=%d\n", N);

	for (uint8_t i = 0; i < N; i++) {

		printf("\t{");
		ShowTM_4_2_PIDValue();
		printf("}\n");
	}

}

void GSSServ4TMHandler::ShowTM_4_2_PIDValue() {

	data_pool_item_type_t data_type;
	data_pool_item_t data_item_raw;
	uint32_t aux_obt;

	uint16_t pid = GetNextUInt16AppDataField();

	uint32_t samples = GetNextUInt32AppDataField();

	data_type = sys_data_pool_item_type(pid);

	printf(" PID=%d,", pid);
	printf(" samples=%d,", samples);

	switch (data_type) {

	case (uint32_item_type):
		data_item_raw.uint32_data = GetNextUInt32AppDataField();
		aux_obt = GetNextUInt32AppDataField();

		printf(" Max Value=");
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);
		printf(", max_time=%d,", aux_obt);

		data_item_raw.uint32_data = GetNextUInt32AppDataField();
		aux_obt = GetNextUInt32AppDataField();

		printf(" Min Value=");
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);

		printf(", min_time=%d,", aux_obt);

		data_item_raw.uint32_data = GetNextUInt32AppDataField();
		printf(" Mean Value=");
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);

		break;

	case (uint8_item_type):
		data_item_raw.uint8_data = GetNextUInt8AppDataField();
		aux_obt = GetNextUInt32AppDataField();

		printf(" Max Value=");
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);
		printf(", max_time=%d,", aux_obt);

		data_item_raw.uint8_data = GetNextUInt8AppDataField();
		aux_obt = GetNextUInt32AppDataField();

		printf(" Min Value=");
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);

		printf(", min_time=%d,", aux_obt);

		data_item_raw.uint8_data = GetNextUInt8AppDataField();
		printf(" Mean Value=");
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);

		break;
	default:
		break;
	}

}

