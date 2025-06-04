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
#include "emu_gss/emu_gss_sys_data_pool.h"

#define TC_12_1_APPDATA_LENGTH 3
#define TC_12_2_APPDATA_LENGTH 3
#define TC_12_5_LIMIT_UINT32_APPDATA_LENGTH 20
#define TC_12_5_LIMIT_UINT8_APPDATA_LENGTH 14
#define TC_12_5_VALUE_UINT8_APPDATA_LENGTH 12
#define TC_12_6_APPDATA_LENGTH 3

EmuGSS_TCProgram12_1::EmuGSS_TCProgram12_1(uint32_t uniTime2YK,
		const char *brief, uint16_t pmonid) :
		EmuGSS_TCProgram(uniTime2YK, 12, 1,
		TC_12_1_APPDATA_LENGTH, brief) {

	mPMONID = pmonid;
	NewProgram(this);
}

void EmuGSS_TCProgram12_1::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPMONID);

}

EmuGSS_TCProgram12_2::EmuGSS_TCProgram12_2(uint32_t uniTime2YK,
		const char *brief, uint16_t pmonid) :
		EmuGSS_TCProgram(uniTime2YK, 12, 2,
		TC_12_2_APPDATA_LENGTH, brief) {

	mPMONID = pmonid;
	NewProgram(this);
}

void EmuGSS_TCProgram12_2::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPMONID);

}

EmuGSS_TCProgram12_5_Limit_UINT32::EmuGSS_TCProgram12_5_Limit_UINT32(
		uint32_t uniTime2YK, const char *brief, uint16_t pmonid, uint16_t pid,
		uint8_t interval, uint8_t repetition, uint32_t lowlimit,
		uint16_t lowlimit_evid, uint32_t highlimit, uint16_t highlimit_evid) :
		EmuGSS_TCProgram(uniTime2YK, 12, 5,
		TC_12_5_LIMIT_UINT32_APPDATA_LENGTH, brief) {

	mPMONID = pmonid;
	mPID = pid;
	mInterval = interval;
	mRepetition = repetition;
	mLowlimit = lowlimit;
	mLowlimitEvID = lowlimit_evid;
	mHighlimit = highlimit;
	mHighlimitEvID = highlimit_evid;
	NewProgram(this);
}

void EmuGSS_TCProgram12_5_Limit_UINT32::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPMONID);
	SetNextUInt16(mPID);
	SetNextUInt8(mInterval);
	SetNextUInt8(mRepetition);
	SetNextUInt8(MonitorCheckTypeLimits);
	SetNextUInt32(mLowlimit);
	SetNextUInt16(mLowlimitEvID);
	SetNextUInt32(mHighlimit);
	SetNextUInt16(mHighlimitEvID);

}

EmuGSS_TCProgram12_5_Limit_UINT8::EmuGSS_TCProgram12_5_Limit_UINT8(
		uint32_t uniTime2YK, const char *brief, uint16_t pmonid, uint16_t pid,
		uint8_t interval, uint8_t repetition, uint8_t lowlimit,
		uint16_t lowlimit_evid, uint8_t highlimit, uint16_t highlimit_evid) :
		EmuGSS_TCProgram(uniTime2YK, 12, 5,
		TC_12_5_LIMIT_UINT8_APPDATA_LENGTH, brief) {

	mPMONID = pmonid;
	mPID = pid;
	mInterval = interval;
	mRepetition = repetition;
	mLowlimit = lowlimit;
	mLowlimitEvID = lowlimit_evid;
	mHighlimit = highlimit;
	mHighlimitEvID = highlimit_evid;
	NewProgram(this);
}

void EmuGSS_TCProgram12_5_Limit_UINT8::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPMONID);
	SetNextUInt16(mPID);
	SetNextUInt8(mInterval);
	SetNextUInt8(mRepetition);
	SetNextUInt8(MonitorCheckTypeLimits);
	SetNextUInt8(mLowlimit);
	SetNextUInt16(mLowlimitEvID);
	SetNextUInt8(mHighlimit);
	SetNextUInt16(mHighlimitEvID);

}

EmuGSS_TCProgram12_5_Value_UINT8::EmuGSS_TCProgram12_5_Value_UINT8(
		uint32_t uniTime2YK, const char *brief, uint16_t pmonid, uint16_t pid,
		uint8_t interval, uint8_t repetition, uint8_t mask,
		uint8_t expectedValue, uint16_t unexpectedValEvID) :
		EmuGSS_TCProgram(uniTime2YK, 12, 5,
		TC_12_5_VALUE_UINT8_APPDATA_LENGTH, brief) {

	mPMONID = pmonid;
	mPID = pid;
	mInterval = interval;
	mRepetition = repetition;
	mMask = mask;
	mExpectedValue = expectedValue;
	mUnexpectedValEvID = unexpectedValEvID;
	NewProgram(this);
}

void EmuGSS_TCProgram12_5_Value_UINT8::BuildTCAppData(
		tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPMONID);
	SetNextUInt16(mPID);
	SetNextUInt8(mInterval);
	SetNextUInt8(mRepetition);
	SetNextUInt8(MonitorCheckTypeExpectedValue);
	SetNextUInt8(mMask);
	SetNextUInt8(mExpectedValue);
	SetNextUInt16(mUnexpectedValEvID);

}

EmuGSS_TCProgram12_6::EmuGSS_TCProgram12_6(uint32_t uniTime2YK,
		const char *brief, uint16_t pmonid) :
		EmuGSS_TCProgram(uniTime2YK, 12, 6,
		TC_12_6_APPDATA_LENGTH, brief) {

	mPMONID = pmonid;
	NewProgram(this);
}

void EmuGSS_TCProgram12_6::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor) {

	SetNextUInt8(1); //N = 1
	SetNextUInt16(mPMONID);

}

//********************************************************+
void EmuGSS_ShowServ12TM(const struct tm_mem_descriptor *pTMDescriptor) {

	GSSServ12TMHandler serv12TMHandler(pTMDescriptor->p_tm_bytes);
	serv12TMHandler.ShowTM();

}

GSSServ12TMHandler::GSSServ12TMHandler(const uint8_t *pTMBytes) :
		GSSTMHandler(pTMBytes) {

}

void GSSServ12TMHandler::ShowTM() {

	switch (mDFHeader.subtype) {
	case (12):
		ShowTM_12_12();
		break;

	}
}

void GSSServ12TMHandler::ShowValueStatus(check_value_status_t status) {

	switch (status) {

	case (MonitorExpValueExpectedValue):
		printf("Expected Value");
		break;
	case (MonitorExpValueUnchecked):
		printf("Unchecked");
		break;
	case (MonitorExpValueUnexpectedValue):
		printf("Unexpected");
		break;
	}
}

void GSSServ12TMHandler::ShowLimitStatus(check_limits_status_t status) {

	switch (status) {

	case (MonitorLimitsWithinLimits):
		printf("Within Limits");
		break;
	case (MonitorLimitsUnchecked):
		printf("Unchecked");
		break;
	case (MonitorLimitsBelowLowLimit):
		printf("Below Low Limit");
		break;
	case (MonitorLimitsAboveHighLimit):
		printf("Above High Limit");
		break;

	}

}

void GSSServ12TMHandler::ShowTM_12_12() {

	uint8_t N;

	//N
	N = GetNextUInt8AppDataField();

	for (uint8_t i = 0; i < N; i++) {

		uint16_t pmonid = GetNextUInt16AppDataField();

		uint16_t pid = GetNextUInt16AppDataField();

		data_pool_item_type_t param_type = sys_data_pool_item_type(pid);

		data_pool_item_t data_mask;
		data_pool_item_t data_item;
		data_pool_item_t data_limit;

		monitor_type_t check_type = (monitor_type_t) GetNextUInt8AppDataField();

		printf(" PMONID = %d PID %d ", pmonid, pid);

		switch (check_type) {

		case (MonitorCheckTypeExpectedValue):
			printf("Check Value");
			break;

		case (MonitorCheckTypeLimits):
			printf("Check Limits");

			break;

		default:
			printf("Error Type");
			break;

		}

		switch (param_type) {

		case (uint32_item_type): {

			if (MonitorCheckTypeExpectedValue == check_type) {

				data_mask.uint32_data = GetNextUInt32AppDataField();

			}

			data_item.uint32_data = GetNextUInt32AppDataField();
			data_limit.uint32_data = GetNextUInt32AppDataField();

			break;
		}
		case (uint8_item_type): {

			if (MonitorCheckTypeExpectedValue == check_type) {

				data_mask.uint8_data = GetNextUInt8AppDataField();

			}

			data_item.uint8_data = GetNextUInt8AppDataField();
			data_limit.uint8_data = GetNextUInt8AppDataField();

		}

		default:
			break;
		}

		if (MonitorCheckTypeExpectedValue == check_type) {

			printf("\n\t\t\t Mask of ");
			GSSSysDataPool::ShowPIDValue(pid, data_mask);
			printf("\n\t\t\t Value of ");
			GSSSysDataPool::ShowPIDValue(pid, data_item);
			printf("\n\t\t\t expected value of ");

			GSSSysDataPool::ShowPIDValue(pid, data_limit);

			check_value_status_t prev_status =
					(check_value_status_t) GetNextUInt8AppDataField();

			printf("\n\t\t\t prev status = ");
			ShowValueStatus(prev_status);

			check_value_status_t current_status =
					(check_value_status_t) GetNextUInt8AppDataField();
			printf("\n\t\t\t current status = ");
			ShowValueStatus(current_status);

		} else {
			printf("\n\t\t\t value of ");
			GSSSysDataPool::ShowPIDValue(pid, data_item);
			printf("\n\t\t\t limit of ");

			GSSSysDataPool::ShowPIDValue(pid, data_limit);

			check_limits_status_t prev_status =
					(check_limits_status_t) GetNextUInt8AppDataField();
			printf("\n\t\t\t prev status = ");
			ShowLimitStatus(prev_status);
			check_limits_status_t current_status =
					(check_limits_status_t) GetNextUInt8AppDataField();
			printf("\n\t\t\t current status =");
			ShowLimitStatus(current_status);

		}

		uint32_t obt = GetNextUInt32AppDataField();

		printf("\n\t\t\t OBT = %d\n", obt);

	}

}
