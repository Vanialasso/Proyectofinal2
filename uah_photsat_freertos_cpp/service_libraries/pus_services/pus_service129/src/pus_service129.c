/*
 * icuasw_pus_service129.c
 *
 *  Created on: Dec 22, 2025
 *      Author: user
 */

#include "public/pus_services_iface_v1.h"
#include "public/attitude_pid_ctrl.h"
#include "public/emu_sat_attitude.h"

#include "pus_service129/aux_pus_service129_exec_tc.h"
#include "pus_service129/aux_pus_service129_observ_mng.h"



void pus_service129_startup() {

	emu_attitude_init();

	pus_service129_observ_mode_exit();

	pus_service129_set_Kp_Ki_Kd(0.07, 0.01, 0.02);

	pus_service129_set_attitude_target(0, 0);

	pus_service129_set_num_of_images(0);

}


void pus_service129_do_attitude_ctrl() {


	//Apply Control Algorithm
	pus_service129_apply_ctrl_algorithm();

	pus_service129_update_system_data_pool();



}

void pus_service129_exec_tc(tc_handler_t *ptc_handler) {

//TODO Complete the execution logic of TC[129,1], TC[129,2] & TC[129,3]

}

void pus_service129_start_observation() {

	pus_service129_observ_mode_entry();

}


void pus_service129_take_image(){

	pus_service129_mng_image_adquisition();

}


bool_t pus_service129_is_last_image(){

	return (0==pus_service129_get_pending_num_of_images());

}

void pus_service129_end_observation() {

	pus_service129_observ_mode_exit();

}


bool_t pus_service129_is_observation_ready(){

	return (pus_service129_attitude_error_under_limits()&&
			pus_service129_get_pending_num_of_images());

}
