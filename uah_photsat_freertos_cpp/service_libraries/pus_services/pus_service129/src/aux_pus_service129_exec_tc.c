/*
 * aux_pus_service129_exec_tc.c
 *
 *  Created on: Feb 13, 2025
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
#include <public/pus_tc_handler.h>
#include "public/config.h"
#include "pus_service129/aux_pus_service129_exec_tc.h"
#include "pus_service129/aux_pus_service129_observ_mng.h"
#include "public/pus_service129.h"
#include "public/pus_service1.h"


bool_t ThetaIsValid(int16_t theta){

	return ((theta <=(45*60))
			&&(theta >=-(45*60)));
}

bool_t NumOfImagesIsValid(uint8_t num_of_images){

	return ((num_of_images<=(10))&&(num_of_images>=1));

}

bool_t PID_Kx_IsValid(float kx){

	return ((kx<=(0.5))&&(kx>=0.0));

}

void pus_service129_exec_TC_129_1(tc_handler_t *ptc_handler) {

	error_code_t error;

	int16_t ThetaPitch;
	int16_t ThetaYaw;
	uint8_t num_of_images;

	//TODO Get Theta Pitch, Theta Yaw and num of images

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {


		if(ThetaIsValid(ThetaPitch)&&ThetaIsValid(ThetaYaw)&&NumOfImagesIsValid(num_of_images)){


						pus_service1_tx_TM_1_3(ptc_handler);

						pus_service129_set_attitude_target(ThetaPitch,ThetaYaw);

						pus_service129_set_num_of_images(num_of_images);

						pus_service1_tx_TM_1_7(ptc_handler);
		}else{


			pus_service1_tx_TM_1_4_Invalid_129_1_Observation(ptc_handler);


		}

	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

void pus_service129_exec_TC_129_2(tc_handler_t *ptc_handler) {

	error_code_t error;

	float Kp;
	float Ki;
	float Kd;

	//TODO Get Kp, Ki and Kd


	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {


		if(PID_Kx_IsValid(Kp)&&PID_Kx_IsValid(Ki)&&PID_Kx_IsValid(Kd)){


						pus_service1_tx_TM_1_3(ptc_handler);

						pus_service129_set_Kp_Ki_Kd(Kp, Ki, Kd);

						pus_service1_tx_TM_1_7(ptc_handler);
		}else{

			pus_service1_tx_TM_1_4_Invalid_129_1_Observation(ptc_handler);


		}

	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

void pus_service129_exec_TC_129_3(tc_handler_t *ptc_handler) {


	pus_service1_tx_TM_1_3(ptc_handler);

	pus_service129_set_Kp_Ki_Kd(0.07, 0.02, 0.01);

	pus_service129_set_attitude_target(0, 0);

	pus_service129_set_num_of_images(0);


	pus_service1_tx_TM_1_7(ptc_handler);

	//free memory
	tc_handler_free_memory(ptc_handler);

}


