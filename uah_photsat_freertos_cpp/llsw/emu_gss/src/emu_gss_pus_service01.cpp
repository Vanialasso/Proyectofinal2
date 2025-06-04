/*
 * emu_gss_pus_service1.cpp
 *
 *  Created on: Jan 4, 2024
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

#include <emu_gss/emu_gss_pus_service01.h>
#include <stdio.h>

#include "public/emu_gss_v1.h"

#include "../../../service_libraries/pus_services/pus_service01/include/public/pus_service1.h"

void EmuGSS_ShowServ1TM(const struct tm_mem_descriptor *pTMDescriptor) {

	GSSServ1TMHandler serv1TMHandler(pTMDescriptor->p_tm_bytes);
	serv1TMHandler.ShowTM();

}

GSSServ1TMHandler::GSSServ1TMHandler(const uint8_t *pTMBytes) :
		GSSTMHandler(pTMBytes) {

}

void GSSServ1TMHandler::ShowTM() {

	//TM-> PACK_ID
	uint16_t pack_id = GetNextUInt16AppDataField();

	//TM-> PACK_SEQ CTRL
	uint16_t pack_seq_ctrl = GetNextUInt16AppDataField();

	switch (mDFHeader.subtype) {
	case (2):
		ShowTM_1_2();
		break;
	case (4):
		ShowTM_1_4();
		break;
	case (8):
		ShowTM_1_8();
		break;
	}
}
void GSSServ1TMHandler::ShowTM_1_2() {

	uint8_t error_code = GetNextUInt8AppDataField();
	switch (error_code) {
	case (TM_1_2_CRC_ERROR_CODE):

		printf(": CRC ERROR ");
		break;

	case (TM_1_2_APID_NOT_VALID_ERROR_CODE):

		printf(": APID ERROR ");
		break;

	case (TM_1_2_SOURCE_ID_NOT_VALID_ERROR_CODE):

		printf(": SOURCE ID ERROR ");
		break;

	case (TM_1_2_TYPE_NOT_VALID_ERROR_CODE):

		printf(": TYPE NOT VALID");

		break;

	case (TM_1_2_SUBTYPE_NOT_VALID_ERROR_CODE):

		printf(": SUBTYPE NOT VALID");

		break;

	}

}

void GSSServ1TMHandler::ShowTM_1_4() {

	uint8_t error_code = GetNextUInt8AppDataField();
	switch (error_code) {
	case (TM_1_4_TC_X_Y_TC_SHORT_PACK_LENGTH):

		printf(": CRC ERROR ");
		break;

	case (TM_1_4_TC_X_Y_TC_NOT_VALID_NUM_OF_INSTR):

		printf(": APID ERROR ");
		break;

	case (TM_1_4_TC_2_X_DEVICE_NOT_VALID):

		printf(": DEVICE NOT VALID");

		break;

	case (TM_1_4_TC_2_X_DEVICE_ADDRESS_NOT_VALID):

		printf(": DEVICE ADDRESS NOT VALID");

		break;
	case (TM_1_4_TC_3_X_INVALID_SID):

		printf(": INVALID SID ");
		break;



	case (TM_1_4_TC_5_X_INVALID_EV_ID):

		printf(": INVALID EvID ");
		break;

	case (TM_1_4_TC_12_X_PMON_UNDEFINED):

		printf(": PMON UNDEFINED");

		break;

	case (TM_1_4_TC_12_X_PMON_DEFINED):

		printf(": PMON UNDEFINED");

		break;
	case (TM_1_4_TC_12_X_PMON_ENABLED):

		printf(": PMON IS ENABLED");

		break;

	case (TM_1_4_TC_12_X_INVALID_PMONID):

		printf(": INVALID PMONID");

		break;
	case (TM_1_4_TC_12_X_INVALID_PMON_DEFINITION):

		printf(": INVALID PMON DEFINITION");

		break;

	case (TM_1_4_TC_19_X_EV_ACTION_IS_ENABLED):

		printf(": EV ACTION IS ENABLED");

		break;

	case (TM_1_4_TC_19_1_EV_ACTION_REJECTED):

		printf(": EV ACTION REJECTED");

		break;

	case (TM_1_4_TC_19_X_EV_ACTION_NOT_DEFINED):

		printf(": EV ACTION NOT DEFINED");

		break;

	case (TM_1_4_TC_20_X_INVALID_PID):

		printf(": INVALID PID ");
		break;

	case (TM_1_4_TC_20_X_PID_READ_ONLY_VIA_TC_20):

		printf(": PID READ_ONLY VIA TC 20_X");
		break;

	default:
		break;
	}
}




void GSSServ1TMHandler::ShowTM_1_8() {

	uint8_t error_code = GetNextUInt8AppDataField();
	switch (error_code) {

	case (TM_1_8_TM_X_Y_TM_EXCEED_LIMIT_APPDATA):

		printf(": TM EXCEED LIMIT APPDATA ");
		break;

	case (TM_1_8_TM_2_1_DEV_COMMAND_EXEC_ERROR):

			printf(": DEV COMMAND EXEC ERROR ");
			break;

	case (TM_1_8_TC_4_1_NOT_FREE_PID_STATS_CONFIG):

		printf(": NOT FREE PID STATS CONFIG");
		break;

	case (TM_1_8_TC_4_7_PID_STATS_UNDEFINED):

		printf(": PID STATS UNDEFINED");

		break;

	case (TM_1_8_TC_6_X_MEM_ACCESS_FAIL):

		printf(": TC 6 2 MEM ACCESS FAILURE");
		break;

	case (TM_1_8_TC_11_X_SCHED_TC_POOL_EMPTY):

			printf(": TC_11_X SCHED TC POOL IS EMPTY");

			break;

	case (TM_1_8_TC_19_1_MAX_EV_ACTIONS_REACHED):

			printf(": MAX EV ACTIONS USED");

			break;



	default:
		break;

	}
}
