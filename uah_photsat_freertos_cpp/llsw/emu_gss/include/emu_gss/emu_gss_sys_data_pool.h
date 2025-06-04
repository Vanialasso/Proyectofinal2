/*
 * emu_gss_sys_data_pool.h
 *
 *  Created on: Nov 7, 2024
 *      Author: opolo70
 */

#ifndef LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_SYS_DATA_POOL_H_
#define LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_SYS_DATA_POOL_H_

#include <stdio.h>

#include "public/emu_gss_v1.h"
#include "public/pus_sys_data_pool.h"

class GSSSysDataPool{

public:

	static void ShowPIDValue(uint16_t pid, data_pool_item_t data_item_raw);


};


#endif /* LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_SYS_DATA_POOL_H_ */
