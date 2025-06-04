/*
 * aux_pus_service129_observ_mng_cpp.cpp
 *
 *  Created on: May 7, 2025
 *      Author: opolo70
 */


#include "public/config.h"
#include "public/pus_tc_handler.h"
#include "public/pus_service1.h"
#include "public/pus_service129.h"
#include "pus_service129/aux_pus_service129_observ_mng.h"
#include "pus_service129/aux_cobserv_mng.h"

#include "public/attitude_pid_ctrl.h"
#include "public/emu_sat_attitude.h"

#define PI 3.14159

//1 rad sec max
#define W_MAX_RAD_PER_SEC   (0.1)


//Instancia del objeto ObservMng de la clase CObservationMng
CObservationMng ObservMng;

uint16_t get_abs_error(int16_t target, int16_t current){
	uint16_t abs_error;

	if(target>current)
		abs_error=target-current;
	else
		abs_error=current-target;

	return abs_error;

}



uint8_t cal_w_control(double w) {
	uint8_t w_control;

	if (w > 0)
		w_control = w / W_MAX_RAD_PER_SEC;
	else
		w_control = -w / W_MAX_RAD_PER_SEC;

	return w_control;

}

extern "C" void pus_service129_set_Kp_Ki_Kd(float kp, float ki, float kd){

	ObservMng.SetPIDParams(kp,ki,kd);

}

extern "C" void pus_service129_set_attitude_target(int16_t theta_pitch, int16_t theta_yaw
		){

	sys_data_pool_set_int16_param(TargetThetaPitchPID, theta_pitch);
	sys_data_pool_set_int16_param(TargetThetaYawPID, theta_yaw);

	ObservMng.SetTargetAttitudeRads((theta_pitch*PI)/(180*60)
									,(theta_yaw*PI)/(180*60));

}

extern "C" void pus_service129_set_num_of_images(uint8_t num_of_images){


	ObservMng.SetPendingImages(num_of_images);

}

extern "C" uint8_t pus_service129_get_pending_num_of_images(){

	return ObservMng.GetPendingImages();

}

extern "C" void pus_service129_observ_mode_entry(){

	sys_data_pool_set_uint8_param(UAH_SAT_OBS_ModeActive, 1);
}

extern "C" void pus_service129_observ_mode_exit(){

	sys_data_pool_set_uint8_param(UAH_SAT_OBS_ModeActive, 0);

}

extern "C" void pus_service129_mng_image_adquisition(){

	ObservMng.MngImageAdquisition();

}


extern "C" void pus_service129_apply_ctrl_algorithm(){

	ObservMng.ApplyCtrlAlgorithm();

}


extern "C" void pus_service129_update_system_data_pool(){

	//Set SystemDataPool parameters

	uah_sat_attitude_t current_attitude=ObservMng.GetCurrentAttitude();

	sys_data_pool_set_int16_param(ThetaPitchPID,
			(int16_t) (current_attitude.ThetaPitch * (180*60)/PI ) );

	sys_data_pool_set_int16_param(ThetaYawPID, (int16_t)
			( current_attitude.ThetaYaw * (180*60)/PI ));


	sys_data_pool_set_uint8_param(ThetaPitch_W_ControlPID,
			cal_w_control(current_attitude.WPitch));
	sys_data_pool_set_uint8_param(ThetaYaw_W_ControlPID,
			cal_w_control(current_attitude.WYaw) );


}

extern "C" bool_t pus_service129_attitude_error_under_limits(){

	//int16_t thetapitch_Error= target_attitude.ThetaPitch=
	int16_t target_pitch_minutes;
	int16_t target_yaw_minutes;
	int16_t pitch_minutes;
	int16_t yaw_minutes;
	uint16_t abs_error_pitch;
	uint16_t abs_error_yaw;

	target_pitch_minutes=sys_data_pool_get_int16_param(TargetThetaPitchPID);
	target_yaw_minutes=sys_data_pool_get_int16_param(TargetThetaYawPID);
	pitch_minutes=sys_data_pool_get_int16_param(ThetaPitchPID);
	yaw_minutes=sys_data_pool_get_int16_param(ThetaYawPID);

	abs_error_pitch=get_abs_error(target_pitch_minutes,pitch_minutes);

	abs_error_yaw=get_abs_error(target_yaw_minutes,yaw_minutes);



	return ((abs_error_pitch<=1)&&(abs_error_yaw<=1));
}



