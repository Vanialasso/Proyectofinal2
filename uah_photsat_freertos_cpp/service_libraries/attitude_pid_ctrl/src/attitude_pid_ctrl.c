/*
 * attitude_pid_ctrl.c
 *
 *  Created on: Apr 28, 2025
 *      Author: opolo70
 */

#include "public/attitude_pid_ctrl.h"

#define PI 3.14159

const double ActuatorPitchLimit = 0.0100; //10 MiliNewtonsxM
const double ActuatorYawLimit = 0.0100; //10 MiliNewtonsxM

float limit_torque(float torque, float limit) {

	float maxTorque = torque;
	if (maxTorque > limit)
		maxTorque = limit;
	else if (maxTorque < (-limit)) {
		maxTorque = -limit;
	}
	return maxTorque;
}

attitude_ctrl_t attitude_pid_ctrl(const uah_sat_attitude_t *p_current_attitude,
		const uah_sat_attitude_t *p_target_attitude,
		const pid_params_t *p_pid_params) {

	attitude_ctrl_t attitude_ctrl;

	static double ErrorThethaPitch = 0;
	static double ErrorThethaYaw = 0;
	static double IntegralErrorThethaPitch = 0;
	static double IntegralErrorThethaYaw = 0;

	double prev_error_theta_pitch = ErrorThethaPitch;
	double prev_error_theta_yaw = ErrorThethaYaw;

	double ThetaPitchTorqueWithoutIntegral;
	double ThetaYawTorqueWithoutIntegral;

	ErrorThethaPitch = (p_target_attitude->ThetaPitch
			- p_current_attitude->ThetaPitch);
	ErrorThethaYaw =
			(p_target_attitude->ThetaYaw - p_current_attitude->ThetaYaw);

	IntegralErrorThethaPitch = 0.7 * ErrorThethaPitch
			+ 0.3 * IntegralErrorThethaPitch;

	IntegralErrorThethaYaw = 0.7 * ErrorThethaYaw
			+ 0.3 * IntegralErrorThethaYaw;

	//Calculate Pitch Torque to apply in the next period
	ThetaPitchTorqueWithoutIntegral = ErrorThethaPitch * p_pid_params->Kp
			- p_current_attitude->WPitch;

	ThetaPitchTorqueWithoutIntegral += (ErrorThethaPitch
			- prev_error_theta_pitch) * p_pid_params->Kd;

	attitude_ctrl.ThetaPitchTorque = ThetaPitchTorqueWithoutIntegral
			+ IntegralErrorThethaPitch * p_pid_params->Ki;

	//Limit Torque
	attitude_ctrl.ThetaPitchTorque = limit_torque(
			attitude_ctrl.ThetaPitchTorque, ActuatorPitchLimit);

	//Anti wind-up
	IntegralErrorThethaPitch = attitude_ctrl.ThetaPitchTorque
			- ThetaPitchTorqueWithoutIntegral;

	//Calculate Yaw Torque to apply in the next period
	ThetaYawTorqueWithoutIntegral = ErrorThethaYaw * p_pid_params->Kp
			- p_current_attitude->WYaw;

	ThetaYawTorqueWithoutIntegral += (ErrorThethaYaw - prev_error_theta_yaw)
			* p_pid_params->Kd;

	attitude_ctrl.ThetaYawTorque = ThetaYawTorqueWithoutIntegral
			+ IntegralErrorThethaYaw * p_pid_params->Ki;

	//Limit Torque
	attitude_ctrl.ThetaYawTorque = limit_torque(attitude_ctrl.ThetaYawTorque,
			ActuatorYawLimit);

	//Anti wind-up
	IntegralErrorThethaYaw = attitude_ctrl.ThetaYawTorque
			- ThetaYawTorqueWithoutIntegral;

	return attitude_ctrl;

}

