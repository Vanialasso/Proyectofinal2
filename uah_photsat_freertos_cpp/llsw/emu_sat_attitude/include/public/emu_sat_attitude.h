/*
 * emu_sat_attitude_ctrl.h
 *
 *  Created on: May 7, 2024
 *      Author: opolo70
 */

#ifndef LLSW_EMU_SAT_ATTITUDE_CTRL_INCLUDE_PUBLIC_SAT_ATTITUDE_CTRL_H_
#define LLSW_EMU_SAT_ATTITUDE_CTRL_INCLUDE_PUBLIC_SAT_ATTITUDE_CTRL_H_



#ifdef __cplusplus
extern "C" {
#endif

struct uah_sat_attitude{
	double ThetaPitch; //Radians
	double ThetaYaw; //Radians
	double WPitch; //Radians per sec
	double WYaw; //Radians per sec
};

typedef struct uah_sat_attitude uah_sat_attitude_t;

/**
* \brief	InitEmuAttitude.
*/

void emu_attitude_init();


/**
* \brief	Emulation Satellite Kinematics apply control .
* @param pitch_torque pitch Torque applied
* @param yaw_torque  Yaw Torque applied
*/

void emu_attitude_apply_ctrl(double pitch_torque, double yaw_torque);


/**
* \brief get Attitude in Emulation Satellite kinematics.
*
* @return the updated Attitude info ([ThetaPitch, ThetaPitch, WPitch, WYaw]).
*/

uah_sat_attitude_t emu_attitude_get_state();




#ifdef __cplusplus
}
#endif


#endif /* LLSW_EMU_SAT_ATTITUDE_CTRL_INCLUDE_PUBLIC_SAT_ATTITUDE_CTRL_H_ */
