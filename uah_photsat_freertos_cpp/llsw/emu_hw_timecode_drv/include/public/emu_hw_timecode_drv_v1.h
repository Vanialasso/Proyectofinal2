/*
 * emu_hw_timecode_drv.h
 *
 *  Created on: Jan 5, 2024
 *      Author: opolo70
 */

#ifndef LLSW_EMU_HW_TIMECODE_DRV_INCLUDE_PUBLIC_EMU_HW_TIMECODE_DRV_V1_H_
#define LLSW_EMU_HW_TIMECODE_DRV_INCLUDE_PUBLIC_EMU_HW_TIMECODE_DRV_V1_H_

#include "public/config.h"
#include "public/basic_types.h"

#define OBT_AFTER_POWER_ON 100000


void EmuHwTimeCodeStartUp();

void EmuHwTimeCodePassSecond();


#ifdef __cplusplus
extern "C" {
#endif

void EmuHwTimeCodeSetNextOBT(uint32_t nextOBTSecns);

uint32_t EmuHwTimeCodeGetCurrentOBT();

uint16_t EmuHwTimeCodeGetCurrentFineTimeOBT();

#ifdef __cplusplus
}
#endif

#endif /* LLSW_EMU_HW_TIMECODE_DRV_INCLUDE_PUBLIC_EMU_HW_TIMECODE_DRV_V1_H_ */
