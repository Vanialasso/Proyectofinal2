/*
 * emu_timecode_hw.cpp
 *
 *  Created on: Jan 5, 2024
 *      Author: opolo70
 */

#include "public/emu_hw_timecode_drv_v1.h"

#ifdef GSS_EMULATION
 void EmuGSS_PassSecond();
 void EmuGSS_PrintCurrentOBT();
#endif

uint32_t EMU_TIME_CODE_HW_NextOBTSecns=OBT_AFTER_POWER_ON + 1;
uint32_t EMU_TIME_CODE_HW_CurrentOBTSecns=OBT_AFTER_POWER_ON;

void EmuHwTimeCodeStartUp(){

#ifdef GSS_EMULATION
	EmuGSS_PrintCurrentOBT();
#endif
}

void EmuHwTimeCodePassSecond(){

	EMU_TIME_CODE_HW_CurrentOBTSecns =EMU_TIME_CODE_HW_NextOBTSecns;
	EMU_TIME_CODE_HW_NextOBTSecns++;

#ifdef GSS_EMULATION

	EmuGSS_PassSecond();

#endif

}

extern "C" void EmuHwTimeCodeSetNextOBT(uint32_t nextOBTSecns){

	EMU_TIME_CODE_HW_NextOBTSecns=nextOBTSecns;
}

extern "C" uint32_t EmuHwTimeCodeGetCurrentOBT(){

	return EMU_TIME_CODE_HW_CurrentOBTSecns;

}

extern "C" uint16_t EmuHwTimeCodeGetCurrentFineTimeOBT(){

	return 0;

}
