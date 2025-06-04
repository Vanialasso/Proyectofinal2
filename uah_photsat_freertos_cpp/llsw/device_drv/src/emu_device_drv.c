/*
 * device_drv.c
 *
 *  Created on: Nov 14, 2024
 *      Author: opolo70
 */



#include "public/device_drv.h"
#include <stdio.h>

//Get sample ADC channel and return value
error_code_t device_drv_on_off(uint32_t on_off_address){

	error_code_t error=0;

	if(0x80000900==on_off_address)
		printf("\t\tDevice Off\n");
	else if(0x8000090A==on_off_address)
		printf("\t\tDevice On\n");
	else
		error=1;

	return error;
}

bool_t device_drv_is_on_off_address_valid(uint32_t on_off_address){

	bool_t is_valid=false;

	if(0x80000900==on_off_address)
		is_valid=true;
	else if(0x8000090A==on_off_address)
		is_valid=true;

	return is_valid;

}




