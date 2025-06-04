/*
 * aux_pus_service12_x_utils.h
 *
 *  Created on: Oct 28, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUS_SERVICE12_AUX_PUS_SERVICE12_X_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUS_SERVICE12_AUX_PUS_SERVICE12_X_UTILS_H_

#include "public/pus_service12.h"

struct param_monitoring_transition{
	uint16_t PMONID;
	uint16_t PID;
	monitor_type_t type;
	data_pool_item_t mask_value;
	data_pool_item_t limit_value;
	data_pool_item_t new_value;
	monitor_status_t prev_status;
	monitor_status_t new_status;
	uint32_t trans_obt;
};

typedef struct param_monitoring_transition param_monitoring_transition_t;


/**
 * \brief set the PMON as unchecked
 * \param validPMONID valid PMON Identifier
 */
void pus_service12_exec_TC_12_2_set_unchecked(uint16_t validPMONID);

/**
 * \brief check param out of high limit.
 * \param PID Parameter identifier
 * \param p_check_def pointer to the check definition
 * \return true if param is out of high limit
 */
bool_t pus_service12_PID_is_out_of_high_limit(uint16_t PID,
		const param_limit_check_definition_t *const p_check_def);

/**
 * \brief check param out of low limit.
 * \param PID Parameter identifier
 * \param p_check_def pointer to the check definition
 * \return true if param is out of low limit
 */
bool_t pus_service12_PID_is_out_of_low_limit(uint16_t PID,
		const param_limit_check_definition_t *const p_check_def);

/**
 * \brief check param has the expected value with the mask applied
 * \param PID Parameter identifier
 * \param p_check_def pointer to the check definition
 * \return true if param has the expected value
 */

bool_t pus_service12_PID_has_expected_masked_value(uint16_t PID,
		const param_value_check_definition_t *const p_mon_config);

/**
 * \brief compare if the statuses are equal .
 * \param type monitoring type
 * \param status1 status1 to compare
 * \param status2 status2 to compare
 * \return true if both statuses are equal
 */
bool_t pus_service12_are_status_equal(monitor_type_t type,
		const monitor_status_t status1, const monitor_status_t status2);

/**
 * \brief manage param out of high limit.
 * return true to indicate the event must be triggered
 * \param pmon_id param monitoring identifier
 * \param p_mon_config pointer to the monitoring configuration
 * \param p_EvID pointer to an Event Identifier
 * \param pfault_info pointer to a parameter out of limits fail info
 * \return true if event must be triggered
 */
bool_t pus_service12_mng_param_out_of_high_limit(
		uint16_t pmon_id,
		param_monitoring_config_t *p_mon_config, uint16_t *p_EvID,
		param_out_of_limits_info_t *pfault_info);

/**
 * \brief manage param out of low limit.
 * return true to indicate the event must be triggered
 * \param pmon_id param monitoring identifier
 * \param p_mon_config pointer to the monitoring configuration
 * \param p_EvID pointer to an Event Identifier
 * \param pfault_info pointer to a parameter out of limits fail info
 * \return true if event must be triggered
 */
bool_t pus_service12_mng_param_out_of_low_limit(
		uint16_t pmon_id,
		param_monitoring_config_t *p_mon_config, uint16_t *p_EvID,
		param_out_of_limits_info_t *pfault_info);

/**
 * \brief manage param unexpected.
 * return true to indicate the event must be triggered
 * \param pmon_id param monitoring identifier
 * \param p_mon_config pointer to the monitoring configuration
 * \param p_EvID pointer to an Event Identifier
 * \param pfault_info pointer to a parameter value fail info
 * \return true if event must be triggered
 */
bool_t pus_service12_mng_param_unexpected_value(
		uint16_t pmon_id,
		param_monitoring_config_t *p_mon_config, uint16_t *p_EvID,
		param_value_fault_info_t *pfault_info);

/**
 * \brief manage new status. If the status is the same than tmp and != current
 * increase the repetition_ctrl, if not, set the repetition_ctrl to zero and
 * update the tmp status
 * if the repetition_ctrl reach the value of repetition, return true to indicate
 * the status transition
 * \param pmon_id parameter monitoring identifier
 * \param p_item_value pointer to the item value
 * \param p_limit_value pointer to the limit value
 * \param p_mon_config pointer to the monitoring configuration
 * \param new_status the new status of the param
 * \return true if status transition is triggered (repetition_ctrl >= repetition)
 */
bool_t pus_service12_mng_new_status(
		uint16_t pmon_id,
		const data_pool_item_t *const p_item_value,
		const data_pool_item_t *const p_limit_value,
		param_monitoring_config_t *p_mon_config,
		monitor_status_t new_status);

/**
 * \brief return true if check limit definition is valid, false if not.
 * \param PID Parameter identifier
 * \param p_limit_check_def the param limit check definition
 * \return true if param limit check is valid (high limit >= low limit)
 */
bool_t pus_service12_mng_is_valid_check_limit_def(uint16_t PID,
		const param_limit_check_definition_t *const p_limit_check_def);


/**
 * \brief add monitoring definition.
 * \param pmon_id parameter monitoring identifier
 * \param p_mon_config pointer to the monitoring configuration
 * \return !=0 if p_mon_config is not valid
 */
error_code_t pus_service12_add_valid_mng_mon_def(uint16_t pmon_id,
		const param_monitoring_config_t *const p_mon_config);

/**
 * \brief get PID limit check definition from a tc
 * \param PID Parameter identifier
 * \param p_limit_check_def the param limit check definition
 * \return !=0 if there is not enough bytes in tc
 */
error_code_t pus_service12_exec_TC_12_X_get_PMON_limit_check_def(
		tc_handler_t *ptc_handler, uint16_t PID,
		param_limit_check_definition_t *p_limit_check_def);

/**
 * \brief get PID value check definition from a tc
 * \param PID Parameter identifier
 * \param p_value_check_def the param value check definition
 * \return !=0 if there is not enough bytes in tc
 */
error_code_t pus_service12_exec_TC_12_X_get_PMON_value_check_def(
		tc_handler_t *ptc_handler, uint16_t PID,
		param_value_check_definition_t *p_value_check_def);

/**
 * \brief add monitoring transition
 * \param  pmon_id parameter monitoring identifier
 * \param  p_item_value pointer to the item value
 * \param  p_limit_value pointer to the limit value
 * \param p_mon_config pointer to the monitoring configuration
 * \param prev_status previous status
 * \param new_status new status
 * \return error != 0 if any pointer is not valid
 */
error_code_t pus_service12_add_monitoring_transition(
		uint16_t pmon_id,
		const data_pool_item_t *const p_item_value,
		const data_pool_item_t *const p_limit_value,
		param_monitoring_config_t *p_mon_config,
		monitor_status_t prev_status,
		monitor_status_t new_status);



#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_PUS_SERVICE12_AUX_PUS_SERVICE12_X_UTILS_H_ */
