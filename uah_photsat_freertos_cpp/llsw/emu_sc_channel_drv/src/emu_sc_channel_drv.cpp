/*
 * sc_channel_drv.c
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


#include <public/sc_channel_drv_v1.h>
#include "public/emu_gss_v1.h"
#include "public/tmtc_dyn_mem.h"
#include "public/tc_queue_drv.h"
#include "public/tc_rate_ctrl.h"
#include "public/cctcmanager_iface_v1.h"
#include "public/edroombp.h"

//RX Emu Channel Mutex
Pr_Mutex RxMutex;
void RXLockFunction(void) {
	RxMutex.Wait();
}
void RXUnlockFunction(void) {
	RxMutex.Signal();
}

//TX Emu Channel Mutex
Pr_Mutex TxMutex;
void TXLockFunction(void) {
	TxMutex.Wait();
}
void TXUnlockFunction(void) {
	TxMutex.Signal();
}
//void SC_Channel_Init(CEDROOMIRQInterface *irqInterface) {
extern "C" void SC_Channel_Init() {
	init_tmtc_pool();

	TCQueue_Init(RXLockFunction, RXUnlockFunction);
}

bool_t SC_Channel_RxData() {

	return false;
}

void SC_Channel_GetNextTC (CDTCMemDescriptor &tcMemDescriptor) {


	uint16_t tcLength;
	uint8_t *pHeadTCMemory;

	pHeadTCMemory = TCQueue_GetHeadTCMemory(tcLength);

	tcMemDescriptor.mTCMemDescriptor.p_tc_bytes = tmtc_pool_alloc();
	tcMemDescriptor.mTCMemDescriptor.tc_num_bytes = tcLength;

	for (int i = 0; i < tcLength; i++) {
		tcMemDescriptor.mTCMemDescriptor.p_tc_bytes[i] = pHeadTCMemory[i];

	}

	TCQueue_HeadTCExtracted();


}

void SC_Channel_HandlePendingTCs(){

	//Trigger other BottomHalfSignal if Queue is not empty
	if(!TCQueue_IsEmpty())

		CCTCManager::EDROOMEventIRQ18.SignalFromTask();
}

extern "C" void SC_Channel_TxTM(const tm_mem_descriptor_t *pTMDescriptor) {


	TXLockFunction();
	EmuGSS_ShowTM(pTMDescriptor);
	TXUnlockFunction();


}



