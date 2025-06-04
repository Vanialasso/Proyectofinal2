/*
 * sys_data_pool_ids.h
 *
 *  Created on: Apr 26, 2025
 *      Author: opolo70
 */

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SYS_DATA_POOL_INCLUDE_PUBLIC_SYS_DATA_POOL_PIDS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SYS_DATA_POOL_INCLUDE_PUBLIC_SYS_DATA_POOL_PIDS_H_


#define UAH_SAT_Volt_5V			0
#define UAH_SAT_Volt_3_3V		1
#define UAH_SAT_Current_5V 		2
#define UAH_SAT_Current_3_3V 	3
#define UAH_SAT_Temp 			4

#define UAH_SAT_OBS_ModeActive	SYS_DATA_POOL_UINT32_PARAMS

#define ThetaPitch_W_ControlPID  (UAH_SAT_OBS_ModeActive +1)
#define ThetaYaw_W_ControlPID  	 (ThetaPitch_W_ControlPID + 1)


#define ThetaPitchPID  (SYS_DATA_POOL_UINT32_PARAMS + SYS_DATA_POOL_UINT8_PARAMS)
#define ThetaYawPID     (ThetaPitchPID +1)

#define TargetThetaPitchPID  (ThetaYawPID +1)
#define TargetThetaYawPID  (TargetThetaPitchPID + 1)




#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SYS_DATA_POOL_INCLUDE_PUBLIC_SYS_DATA_POOL_PIDS_H_ */
