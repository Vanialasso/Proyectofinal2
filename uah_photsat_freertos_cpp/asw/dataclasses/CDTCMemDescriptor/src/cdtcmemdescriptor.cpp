/*
 * cdtcdescriptor.c
 *
 *  Created on: Oct 7, 2024
 *      Author: opolo70
 */


#include "public/cdtcmemdescriptor.h"
#include <public/sc_channel_drv_v1.h>

	bool CDTCMemDescriptor::HandleIRQ(){

		return SC_Channel_RxData();

	}


