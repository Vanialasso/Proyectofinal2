/*
 * attitude_pid_ctrl.h
 *
 *  Created on: Apr 28, 2025
 *      Author: opolo70
 */

#ifndef SERVICE_LIBRARIES_ATTITUDE_PID_CTRL_INCLUDE_PUBLIC_ATTITUDE_PID_CTRL_H_
#define SERVICE_LIBRARIES_ATTITUDE_PID_CTRL_INCLUDE_PUBLIC_ATTITUDE_PID_CTRL_H_

#include "public/basic_types.h"
#include "public/config.h"

#include "../../../../llsw/emu_sat_attitude/include/public/emu_sat_attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

struct attitude_ctrl{
	double ThetaPitchTorque;
	double ThetaYawTorque;
};

typedef struct attitude_ctrl attitude_ctrl_t;

struct pid_params{
	double Kp;
	double Ki;
	double Kd;
};

typedef struct pid_params pid_params_t;


/**
* \brief	calculate Satellite Attitude control.
* @param current_attitude current attitude
*
* @return the attitude control to apply
*/

attitude_ctrl_t attitude_pid_ctrl( const uah_sat_attitude_t *p_current_attitude,
								 const uah_sat_attitude_t *p_target_attitude,
								 const pid_params_t *p_pid_params);




#ifdef __cplusplus
}
#endif



#endif /* SERVICE_LIBRARIES_ATTITUDE_PID_CTRL_INCLUDE_PUBLIC_ATTITUDE_PID_CTRL_H_ */
