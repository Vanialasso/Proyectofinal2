/*
 * emu_uah_exp_cinematics.cpp
 *
 *  Created on: May 7, 2024
 *      Author: opolo70
 */

#include "../include/public/emu_sat_attitude.h"

#include "public/edroombp.h"

#define CubeSat_8U_MassKg 8.0
#define CubeSat_8U_SideLength  0.2

#define CubeSatMomentOfInertia ((CubeSat_8U_MassKg/6.0)*(CubeSat_8U_SideLength*CubeSat_8U_SideLength))

//Sat Attitude
uah_sat_attitude_t UAHSatAttitude;

double Pitch_Torquei_1;
double Yaw_Torquei_1;

//Time of this information
Pr_Time UAHSatRefTime;

extern "C" void emu_attitude_init() {

	UAHSatRefTime.GetTime();
	UAHSatAttitude.WPitch = 0.0;
	UAHSatAttitude.WYaw = 0.0;
	UAHSatAttitude.ThetaPitch = 0.0;
	UAHSatAttitude.ThetaYaw = 0.0;
	Pitch_Torquei_1=0;
	Yaw_Torquei_1=0;
}

extern "C" void emu_attitude_apply_ctrl(double pitch_torque, double yaw_torque){

	Pitch_Torquei_1=pitch_torque;
	Yaw_Torquei_1=yaw_torque;


}

extern "C" uah_sat_attitude_t emu_attitude_get_state() {

	Pr_Time aux;
	double interval;
	double WPitch_i_1;
	double WYaw_i_1;

	//Calculate Interval from previous invocation
	aux.GetTime();
	aux -= UAHSatRefTime;
	interval = aux.FineTime();

	//Update reference time (It will be used in the next invocation)
	UAHSatRefTime.GetTime();

	//Save W at i_1
	WPitch_i_1 = UAHSatAttitude.WPitch;
	WYaw_i_1 = UAHSatAttitude.WYaw;

	//Update  WPitch and WYaw W= Torque/I *t, where I=Moment of Inertia
	UAHSatAttitude.WPitch += (Pitch_Torquei_1 / CubeSatMomentOfInertia)
			* interval;
	UAHSatAttitude.WYaw += (Yaw_Torquei_1 / CubeSatMomentOfInertia)
			* interval;

	//Upadate Attitude
	UAHSatAttitude.ThetaPitch = UAHSatAttitude.ThetaPitch
			+ ((UAHSatAttitude.WPitch + WPitch_i_1)*interval)/2.0 ;
	UAHSatAttitude.ThetaYaw = UAHSatAttitude.ThetaYaw
			+ ((UAHSatAttitude.WYaw + WYaw_i_1)*interval)/2.0 ;

	//return Attitude
	return UAHSatAttitude;
}
