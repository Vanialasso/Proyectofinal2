/*
 * emu_sc_channel_drv_slib_v1.h
 *
 *  Created on: Dec 20, 2016
 *      Author: user
 */

#ifndef EMU_GSS_V1_H_
#define EMU_GSS_V1_H_

//#include <public/ccepdmanager_iface_v1.h>
//#include <public/ccsensormanager_iface_v1.h>


#include "public/cdtchandler.h"
#include "public/basic_types.h"
#include "public/tmtc_dyn_mem.h"
#include "public/ccsds_pus.h"
#include "../emu_gss/emu_gss_tm_handler.h"
#include "../emu_gss/emu_gss_tc_program.h"

#include "../emu_gss/emu_gss_pus_service01.h"
#include "../emu_gss/emu_gss_pus_service02.h"
#include "../emu_gss/emu_gss_pus_service03.h"
#include "../emu_gss/emu_gss_pus_service04.h"
#include "../emu_gss/emu_gss_pus_service05.h"
//#include "../emu_gss/emu_gss_pus_service06.h"
#include "../emu_gss/emu_gss_pus_service09.h"
#include "../emu_gss/emu_gss_pus_service12.h"
#include "../emu_gss/emu_gss_pus_service17.h"
#include "../emu_gss/emu_gss_pus_service19.h"
#include "../emu_gss/emu_gss_pus_service20.h"
#include "../emu_gss/emu_gss_pus_service17.h"

#include "../emu_gss/emu_gss_pus_service128.h"
#include "../emu_gss/emu_gss_pus_service129.h"

void EmuGSS_ShowTM(const struct tm_mem_descriptor * pTMDescriptor);

void EmuGSS_PrintCurrentOBT();

void EmuGSS_PassSecond();


void EmuGSS_SendProgrammedTCs();




#endif /* EMU_SC_CHANNEL_DRV_SLIB_V1_H_ */
