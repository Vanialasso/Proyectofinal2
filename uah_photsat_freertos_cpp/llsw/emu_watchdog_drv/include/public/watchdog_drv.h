/*
 * watchdog_drv.h
 *
 *  Created on: Nov 29, 2024
 *      Author: opolo70
 */

#ifndef LLSW_EMU_WATCHDOG_DRV_INCLUDE_PUBLIC_WATCHDOG_DRV_H_
#define LLSW_EMU_WATCHDOG_DRV_INCLUDE_PUBLIC_WATCHDOG_DRV_H_

#include "public/config.h"
#include "public/basic_types.h"


#ifdef __cplusplus
extern "C" {
#endif

void watchdog_drv_startup(uint16_t secns);

void watchdog_drv_disable_signal();

void watchdog_drv_signal();

#ifdef __cplusplus
}
#endif



#endif /* LLSW_EMU_WATCHDOG_DRV_INCLUDE_PUBLIC_WATCHDOG_DRV_H_ */
