/*
 * emu_gss.cpp
 *
 *  Created on: Jan 3, 2024
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

#include <stdio.h>

#include "public/emu_gss_v1.h"

#include <public/cctcmanager_iface_v1.h>
#include "public/emu_hw_timecode_drv_v1.h"
#include "public/tc_queue_drv.h"

#include "public/serialize.h"


void EmuGSS_ShowTM(const struct tm_mem_descriptor *pTMDescriptor) {

	//Get TM Type

	uint8_t tm_type = GSSTMHandler::GetPUSType(pTMDescriptor);
	uint8_t tm_subtype = GSSTMHandler::GetPUSSubtype(pTMDescriptor);

	printf("GSS Rx TM [%i,%i]", tm_type, tm_subtype);

	switch (tm_type) {

	case (1):

		EmuGSS_ShowServ1TM(pTMDescriptor);

		break;

	case (3):
		EmuGSS_ShowServ3TM(pTMDescriptor);
		break;

	case (4):
		EmuGSS_ShowServ4TM(pTMDescriptor);
		break;

	case (5):
		EmuGSS_ShowServ5TM(pTMDescriptor);
				break;
	case (12):
		EmuGSS_ShowServ12TM(pTMDescriptor);
				break;
	case (20):
		EmuGSS_ShowServ20TM(pTMDescriptor);
				break;
	};

	printf("\n");

}

void EmuGSS_PrintCurrentOBT() {

	printf("\nCurrent OBT is = %i\n", EmuHwTimeCodeGetCurrentOBT());

}

void EmuGSS_PassSecond() {

//Emulate Hw TimeCode reception TO_DO

	EmuGSS_PrintCurrentOBT();

	EmuGSS_SendProgrammedTCs();

}

void EmuGSS_RxCurrentTC(CDTCMemDescriptor * pTCDescriptor){



}

void EmuGSS_SendProgrammedTCs() {

	uint32_t currentOBT = EmuHwTimeCodeGetCurrentOBT();

	bool tc_programmed=false;
	while (EmuGSS_TCProgram::SetCurrentTCProgramed(currentOBT)) {

		printf("\nGSS Tx TC[%i,%i] ",
				(EmuGSS_TCProgram::sCurrentTCProgram)->GetType(),
				(EmuGSS_TCProgram::sCurrentTCProgram)->GetSubtype());

		printf("%s",EmuGSS_TCProgram::sCurrentTCProgram->GetBrief());
		printf("\n");

		//TC descriptor
		tc_mem_descriptor_t tcDescriptor;

		EmuGSS_TCProgram *ptc_program = EmuGSS_TCProgram::GetCurrentTCProgramed();

		tcDescriptor.p_tc_bytes = TCQueue_GetTailTCMemory();

		ptc_program->BuildTC(tcDescriptor);

		TCQueue_TailTCCompleted(tcDescriptor.tc_num_bytes);

		tc_programmed=true;


	}

	if(tc_programmed){

		CCTCManager::EDROOMEventIRQ18.SignalFromTask();

	}

}


