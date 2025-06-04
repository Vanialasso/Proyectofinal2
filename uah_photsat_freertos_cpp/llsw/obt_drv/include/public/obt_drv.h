/*
 * obt_drv.h
 *
 *  Created on: Oct 16, 2024
 *      Author: opolo70
 */

#ifndef LLSW_OBT_DRV_INCLUDE_OBT_DRV_H_
#define LLSW_OBT_DRV_INCLUDE_OBT_DRV_H_

#include "public/config.h"
#include "public/basic_types.h"

#ifdef __cplusplus
extern "C" {
#endif
uint32_t obt_drv_get_current_obt();

void obt_drv_set_current_obt(uint32_t next_obt);

uint16_t obt_drv_get_current_finetime_16bits();

#ifdef __cplusplus
}
#endif
#endif /* LLSW_OBT_DRV_INCLUDE_OBT_DRV_H_ */
