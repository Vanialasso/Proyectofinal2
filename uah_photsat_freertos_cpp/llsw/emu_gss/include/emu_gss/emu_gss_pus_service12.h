/*
 * emu_sc_pus_service12.h
 *
 *  Created on: Jan 16, 2024
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

#ifndef EMU_SC_PUS_SERVICE12_H_
#define EMU_SC_PUS_SERVICE12_H_

#include "public/pus_service12.h"

class EmuGSS_TCProgram12_1: public EmuGSS_TCProgram {

	uint16_t mPMONID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram12_1(uint32_t uniTime2YK, const char *brief, uint16_t PMONID);

};

class EmuGSS_TCProgram12_2: public EmuGSS_TCProgram {

	uint16_t mPMONID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram12_2(uint32_t uniTime2YK, const char *brief, uint16_t PMONID);

};

class EmuGSS_TCProgram12_5_Limit_UINT32: public EmuGSS_TCProgram {

	uint16_t mPMONID;
	uint16_t mPID;
	uint8_t mInterval;
	uint8_t mRepetition;
	uint32_t mLowlimit;
	uint16_t mLowlimitEvID;
	uint32_t mHighlimit;
	uint16_t mHighlimitEvID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram12_5_Limit_UINT32(uint32_t uniTime2YK, const char *brief, uint16_t pmonid,
			uint16_t pid, uint8_t interval, uint8_t repetition, uint32_t lowlimit,
			uint16_t lowlimit_evid, uint32_t highlimit, uint16_t highlimit_evid);

};

class EmuGSS_TCProgram12_5_Limit_UINT8: public EmuGSS_TCProgram {

	uint16_t mPMONID;
	uint16_t mPID;
	uint8_t mInterval;
	uint8_t mRepetition;
	uint8_t mLowlimit;
	uint16_t mLowlimitEvID;
	uint8_t mHighlimit;
	uint16_t mHighlimitEvID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram12_5_Limit_UINT8(uint32_t uniTime2YK, const char *brief, uint16_t pmonid,
			uint16_t pid, uint8_t interval, uint8_t repetition, uint8_t lowlimit,
			uint16_t lowlimit_evid, uint8_t highlimit, uint16_t highlimit_evid);

};

class EmuGSS_TCProgram12_5_Value_UINT8: public EmuGSS_TCProgram {

	uint16_t mPMONID;
	uint16_t mPID;
	uint8_t mInterval;
	uint8_t mRepetition;
	uint8_t mMask;
	uint8_t mExpectedValue;
	uint16_t mUnexpectedValEvID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram12_5_Value_UINT8(uint32_t uniTime2YK, const char *brief, uint16_t pmonid,
			uint16_t pid, uint8_t interval, uint8_t repetition,
			uint8_t mask,
			uint8_t expectedValue,
			uint16_t unexpectedValEvID);

};


class EmuGSS_TCProgram12_6: public EmuGSS_TCProgram {

	uint16_t mPMONID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram12_6(uint32_t uniTime2YK, const char *brief, uint16_t PMONID);

};

void EmuGSS_ShowServ12TM(const struct tm_mem_descriptor *pTMDescriptor);

class GSSServ12TMHandler: public GSSTMHandler {

	friend void EmuGSS_ShowServ12TM(const struct tm_mem_descriptor*);

protected:

	GSSServ12TMHandler(const uint8_t *pTMBytes);

	void ShowTM();

	void ShowTM_12_12();

	void ShowValueStatus(check_value_status_t status);

	void ShowLimitStatus(check_limits_status_t status);

};

#endif /* EMU_SC_PUS_SERVICE12_H_ */
