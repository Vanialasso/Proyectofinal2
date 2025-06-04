/*
 * cdevaction.cpp
 *
 *  Created on: Nov 12, 2024
 *      Author: opolo70
 */

#include "public/cdevaction_iface_v1.h"
#include "public/pus_service19.h"

void CDEvAction::ExtractEvActionFromQueue(){

	mTCHandler.mTCHandler.raw_tc_mem_descriptor.p_tc_bytes=tmtc_pool_alloc();

	pus_service19_extract_next_ev_action(&mTCHandler.mTCHandler);

	tc_handler_build_from_descriptor(&mTCHandler.mTCHandler,
			mTCHandler.mTCHandler.raw_tc_mem_descriptor);


}


