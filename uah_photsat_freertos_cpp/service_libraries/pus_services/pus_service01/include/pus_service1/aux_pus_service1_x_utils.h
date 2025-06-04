/*
 * aux_pus_service1_utils.h
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE01_INCLUDE_PUS_SERVICE1_AUX_PUS_SERVICE1_X_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE01_INCLUDE_PUS_SERVICE1_AUX_PUS_SERVICE1_X_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <public/pus_tm_handler.h>
#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_accept_report.h"
#include "public/pus_tc_accept_report.h"
#include "public/pus_tc_handler.h"

enum tc_verif_stage {
	TCVerifStageAcceptation,
	TCVerifStageExecStart,
	TCVerifStageExecProgress,
	TCVerifStageExecCompletion,
};
typedef enum tc_verif_stage tc_verif_stage_t;

error_code_t pus_service1_build_TM_1_X_reqID(tc_handler_t *ptc_handler,
		tm_handler_t *p_tm_handler);

error_code_t pus_service1_tx_TM_1_X_reqID(tc_handler_t *ptc_handler,
		uint8_t type, uint8_t subtype);

error_code_t pus_service1_build_TM_1_2_failure_notice(tc_handler_t *ptc_handler,
		tm_handler_t *pvalid_tm_handler, tc_accept_report_t tc_accept_report);

error_code_t pus_service1_tx_TM_1_X_no_failure_data(tc_handler_t *ptc_handler,
		tc_verif_stage_t verif_stage, uint8_t failure_code);

error_code_t pus_service1_tx_TM_1_X_uint8_failure_data(
		tc_handler_t *ptc_handler, tc_verif_stage_t verif_stage,
		uint8_t failure_code, uint8_t failure_data);

error_code_t pus_service1_tx_TM_1_X_uint16_failure_data(
		tc_handler_t *ptc_handler, tc_verif_stage_t verif_stage,
		uint8_t failure_code, uint16_t failure_data);

error_code_t pus_service1_tx_TM_1_X_uint32_failure_data(
		tc_handler_t *ptc_handler, tc_verif_stage_t verif_stage,
		uint8_t failure_code, uint32_t failure_data);

error_code_t pus_service1_tx_TM_1_X_uint8_uint32_failure_data(
		tc_handler_t *ptc_handler, tc_verif_stage_t verif_stage,
		uint8_t failure_code, uint8_t failure_data1, uint32_t failure_data2);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE01_INCLUDE_PUS_SERVICE1_AUX_PUS_SERVICE1_X_UTILS_H_ */
