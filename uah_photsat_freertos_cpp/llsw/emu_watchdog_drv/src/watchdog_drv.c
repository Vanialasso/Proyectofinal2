/*
 * watchdog_drv.c
 *
 *  Created on: Nov 29, 2024
 *      Author: opolo70
 */

#include "public/watchdog_drv.h"
#include "public/basic_types.h"
#include <stdlib.h>
#include <stdio.h>

void watchdog_drv_startup(uint16_t secns){



}

void watchdog_drv_disable_signal(){

	printf("REBOOT");
	exit(0);
}

void watchdog_drv_signal(){



}
