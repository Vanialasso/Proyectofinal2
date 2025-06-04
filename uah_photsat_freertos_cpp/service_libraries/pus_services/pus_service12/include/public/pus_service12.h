/*
 * pus_service12.h
 *
 *  Created on: Oct 27, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUBLIC_PUS_SERVICE12_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUBLIC_PUS_SERVICE12_H_

#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_handler.h"
#include "public/pus_tm_handler.h"
#include "public/pus_sys_data_pool.h"
#include "public/pus_service5.h"

#ifdef __cplusplus
extern "C" {
#endif


#define PUS_SERVICE12_MAX_NUM_OF_PMONS 16

#define PUS_SERVICE12_MAX_NUM_OF_TRANS 1

enum monitor_type{

	MonitorCheckTypeExpectedValue=0,
	MonitorCheckTypeLimits=1,
//	MonitorCheckTypeDelta=2,
	MonitorFree=0xFF

};

typedef enum monitor_type monitor_type_t;

enum check_limits_status{

	MonitorLimitsWithinLimits=0,
	MonitorLimitsUnchecked=1,
	//MonitorLimitsInvalid=2,
	MonitorLimitsBelowLowLimit=3,
	MonitorLimitsAboveHighLimit=4

};

typedef enum check_limits_status check_limits_status_t;


enum check_value_status{

	MonitorExpValueExpectedValue=0,
	MonitorExpValueUnchecked=1,
	//MonitorValueInvalidValue=2,
	MonitorExpValueUnexpectedValue=3

};

typedef enum check_value_status check_value_status_t;

struct param_limit_check_definition{
  uint16_t high_limit_EvID;
  data_pool_item_t high_limit;
  uint16_t low_limit_EvID;
  data_pool_item_t  low_limit;
};

typedef struct param_limit_check_definition param_limit_check_definition_t;

struct param_value_check_definition{
  data_pool_item_t mask_value;
  data_pool_item_t expected_value;
  uint16_t EvID;
};

typedef struct param_value_check_definition param_value_check_definition_t;

union monitor_definition{

	param_limit_check_definition_t limit_check_def;
	param_value_check_definition_t value_check_def;
};

typedef union monitor_definition monitor_definition_t;

union monitor_status{

	check_limits_status_t limit_status;
	check_value_status_t value_status;
};

typedef union monitor_status monitor_status_t;



struct param_monitoring_config{
	uint16_t PID;
	monitor_type_t type;
	bool_t  enabled;
	monitor_definition_t monitor_def;
	uint8_t repetition;
	uint8_t repetition_ctrl;
	uint8_t interval;
	uint8_t interval_ctrl;
	uint32_t transition_obt;
	monitor_status_t temp_status;
	monitor_status_t current_status;

};

typedef struct param_monitoring_config param_monitoring_config_t;


/**
 * \brief Init the pus_service12
 */
void pus_service12_startup();

/**
 * \brief executes a TC[12,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service12_exec_tc(tc_handler_t *ptc_handler);

/**
 * \brief cal the total number of PMON
 * \return the total number of PMON
 */
uint16_t pus_service12_get_PMON_COUNT();

/**
 * \brief cal the total number of PMON
 * \return the total number of PMON
 */
bool_t pus_service12_is_PMON_enabled(uint16_t PMONID);


/**
 * \brief cal the total number of PMON
 * \return the total number of PMON
 */
monitor_type_t pus_service12_get_PMON_type(uint16_t PMONID);

/**
 * \brief Do a param limits monitoring defined in PMONID
 * \param p_EvID pointer to an Event Identifier
 * \param pfault_info pointer to a parameter out of limits fail info
 * \return true if the monitoring has triggered an Event
 */
bool_t pus_service12_do_param_limits_monitoring(uint16_t PMONID
		,uint16_t *p_EvID
		,param_out_of_limits_info_t *pfault_info);


/**
 * \brief Do a param check value monitoring defined in PMONID
 * \param p_EvID pointer to an Event Identifier
 * \param pfault_info pointer to a parameter check value fail info
 * \return true if the monitoring has triggered an Event
 */
bool_t pus_service12_do_param_check_value_monitoring(uint16_t PMONID
		,uint16_t *p_EvID
		,param_value_fault_info_t *pfault_info);

#ifdef __cplusplus
}
#endif


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUBLIC_PUS_SERVICE12_H_ */
