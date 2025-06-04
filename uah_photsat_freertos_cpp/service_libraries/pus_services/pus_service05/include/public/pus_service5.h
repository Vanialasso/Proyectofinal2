/*
 * pus_service5.h
 *
 *  Created on: Oct 25, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE5_INCLUDE_PUS_SERVICE5_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE5_INCLUDE_PUS_SERVICE5_H_

#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_handler.h"
#include "public/pus_tm_handler.h"
#include "public/pus_sys_data_pool.h"

#ifdef __cplusplus
extern "C" {
#endif

//Informative EvIDs 0x1001,0x1002,0x1003
#define PUS_5_InformativeEvents 			3

//Low Severity Anomaly EvIDs None
#define PUS_5_LowSeverityAnomalyEvents 	0

//Medium Severity Anomaly EvIDs None
#define PUS_5_MediumSeverityAnomalyEvents 0

//High Severity Anomaly EvIDs 0x4001,0x4002,0x4003, 0x4004,0x4005
//#define PUS_5_HighSeverityAnomalyEvents 	5

//High Severity Anomaly EvIDs 0x4001,0x4002,0x4003
#define PUS_5_HighSeverityAnomalyEvents 	3

enum pus_service5_TM_5_X_EvIDs{TM_5_1_START_UP=0x1001,
	TM_5_1_SENSOR_ON=0x1002,TM_5_1_SENSOR_OFF=0x1003,
	TM_5_4_PARAM_OUT_OF_LOW_LIMIT=0x4001,TM_5_4_PARAM_OUT_OF_HIGH_LIMIT=0x4002,
	TM_5_4_PARAM_VALUE_NOT_EXPECTED=0x4003};

struct param_out_of_limits_info{
		uint16_t PID;
		data_pool_item_t PIDValue;
		data_pool_item_t PIDLimit;
};

typedef struct param_out_of_limits_info param_out_of_limits_info_t;

struct param_value_fault_info{
		uint16_t PID;
		data_pool_item_t PIDValue;
		data_pool_item_t PIDMask;
		data_pool_item_t PIDExpectedValue;
};

typedef struct param_value_fault_info param_value_fault_info_t;

/* brief enum that defines Event types
 * 0 if is not a valid EvID identifier, 1 for valid informative Events,
 * 2 for valid Low Severity Anomaly, 3 for a valid medium Severity Anomaly,
  *4 for a valid high Severity Anomaly
  *
  */
enum ev_type{
	EvNotValid=0, EvTypeInformative=1, EvTypeLowSevAnomaly=2
	,EvTypeMediumSevAnomaly=3,EvTypeHighSevAnomaly=4
};

typedef enum ev_type ev_type_t;



/**
 * \brief get the EvID type
 * \param EvID Event Identifier
 * \return the type of the event
 */
ev_type_t pus_service5_get_EvID_type(uint16_t EvID);


/**
 * \brief return the validity of EvID
 * \param EvID Event Identifier
 * \return true if the EvID is valid
 */
bool_t pus_service5_is_EvID_valid(uint16_t EvID);

/**
 * \brief return the enabled status of an event
 * \param EvID Event Identifier
 * return true if EvID enabled and EvID is valid
 */
bool_t pus_service5_is_EvID_enabled(uint16_t EvID);


/**
 * \brief executes a TC[5,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service5_exec_tc(tc_handler_t *ptc_handler);

/**
 * \brief TX a TM[5,1] Power On
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_5_1_start_up();

/**
 * \brief TX a TM[5,X] Parameter Monitoring
 * \param EvID Event Identifier
 * \param pfault_info pointer to a parameter out of limit info
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_5_X_param_out_of_limit(uint16_t EvID,
		param_out_of_limits_info_t *pfault_info);

/**
 * \brief TX a TM[5,X] Parameter Monitoring
 * \param EvID Event Identifier
 * \param pfault_info pointer to a parameter check value fail info
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_5_X_param_check_value_fail(uint16_t EvID
		,param_value_fault_info_t *pfault_info);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE5_INCLUDE_PUS_SERVICE5_H_ */
