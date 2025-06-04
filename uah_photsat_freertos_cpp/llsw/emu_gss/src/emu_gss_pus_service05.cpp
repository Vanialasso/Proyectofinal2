/*
 * emu_sc_pus_service3.cpp
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
#include "emu_gss/emu_gss_sys_data_pool.h"
#include "../../../service_libraries/pus_services/pus_service05/include/public/pus_service5.h"

#define TC_5_5_APPDATA_LENGTH 3
#define TC_5_6_APPDATA_LENGTH 3

EmuGSS_TCProgram5_5::EmuGSS_TCProgram5_5(uint32_t uniTime2YK, const char *brief,
		uint16_t EvID) :
		EmuGSS_TCProgram(uniTime2YK, 5, 5,
		TC_5_5_APPDATA_LENGTH, brief) {

	mEvID = EvID;
	NewProgram(this);
}

void EmuGSS_TCProgram5_5::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mEvID);

}

EmuGSS_TCProgram5_6::EmuGSS_TCProgram5_6(uint32_t uniTime2YK, const char *brief,
		uint16_t EvID) :
		EmuGSS_TCProgram(uniTime2YK, 5, 6,
		TC_5_6_APPDATA_LENGTH, brief) {

	mEvID = EvID;
	NewProgram(this);
}

void EmuGSS_TCProgram5_6::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mEvID);

}

//********************************************************+
void EmuGSS_ShowServ5TM(const struct tm_mem_descriptor *pTMDescriptor) {

	GSSServ5TMHandler serv5TMHandler(pTMDescriptor->p_tm_bytes);
	serv5TMHandler.ShowTM();

}

GSSServ5TMHandler::GSSServ5TMHandler(const uint8_t *pTMBytes) :
		GSSTMHandler(pTMBytes) {

}

void GSSServ5TMHandler::ShowTM() {

	switch (mDFHeader.subtype) {
	case (1):
		ShowTM_5_1();
		break;
	case (2):
		ShowTM_5_2();
		break;
	case (3):
		ShowTM_5_3();
		break;
	case (4):
		ShowTM_5_4();
		break;

	}
}

void GSSServ5TMHandler::ShowTM_5_1() {

	uint16_t EvId = GetNextUInt16AppDataField();

	printf(" Informative EvID = 0x%X ", EvId);

	switch (EvId) {

	case (TM_5_1_START_UP):

		printf(": START_UP ");
		break;

	case (TM_5_1_SENSOR_ON):

		printf(": SENSORS ON ");
		break;

	case (TM_5_1_SENSOR_OFF):

		printf(": SENSORS OFF ");
		break;
	default:
		break;
	}
	printf("\n");
}

void GSSServ5TMHandler::ShowTM_5_2() {

	uint16_t EvID = GetNextUInt16AppDataField();

	printf(" Low Severity Anomaly EvID = 0x%X \n", EvID);

}

void GSSServ5TMHandler::ShowTM_5_3() {

	uint16_t EvID = GetNextUInt16AppDataField();

	printf(" Medium Severity Anomaly EvID 0x%X \n", EvID);

}

void GSSServ5TMHandler::ShowTM_5_4() {

	uint16_t EvID = GetNextUInt16AppDataField();

	printf(" High Severity Anomaly EvID 0x%X ", EvID);

	switch (EvID) {
	case (TM_5_4_PARAM_OUT_OF_LOW_LIMIT):
	case (TM_5_4_PARAM_OUT_OF_HIGH_LIMIT):
		uint16_t pid = GetNextUInt16AppDataField();

		data_pool_item_type_t data_type;
		data_pool_item_t data_item_raw;
		data_pool_item_t data_limit_raw;

		data_type = sys_data_pool_item_type(pid);

		switch (data_type) {

		case (uint32_item_type):
			data_item_raw.uint32_data = GetNextUInt32AppDataField();
			data_limit_raw.uint32_data = GetNextUInt32AppDataField();
			break;

		case (uint8_item_type):
			data_item_raw.uint8_data = GetNextUInt8AppDataField();
			data_limit_raw.uint8_data = GetNextUInt8AppDataField();
			break;
		default:

			break;
		}

		printf(" PID %d = ", pid);
		GSSSysDataPool::ShowPIDValue(pid, data_item_raw);

		if (TM_5_4_PARAM_OUT_OF_LOW_LIMIT == EvID)
			printf(" Low Limit = ");
		else
			printf(" High Limit = ");

		GSSSysDataPool::ShowPIDValue(pid, data_limit_raw);

		printf("\n");

		break;

	}
}
