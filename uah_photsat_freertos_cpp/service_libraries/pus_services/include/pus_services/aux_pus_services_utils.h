/*
 * aux_pus_services_utils.h
 *
 *  Created on: Oct 26, 2024
 *
 *  Created on: Dec 21, 2023
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_INCLUDE_PUS_SERVICES_AUX_PUS_SERVICES_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_INCLUDE_PUS_SERVICES_AUX_PUS_SERVICES_UTILS_H_

#include <public/pus_tm_handler.h>
#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_accept_report.h"
#include "public/pus_tc_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief write the parameter value in a TM[X,Y] telemetry
 * \param ptm_handler pointer to a tm handler
 * \param param_id a parameter identifier
 * \return 0 if no error
 */
error_code_t pus_services_TM_X_Y_write_PIDValue(tm_handler_t *ptm_handler,
		uint16_t param_id);

/**
 * \brief write the saved parameter value in a TM[X,Y] telemetry
 * \param ptm_handler pointer to a tm handler
 * \param param_id a parameter identifier
 * \param p_pid_value param value pointer
 * \return 0 if no error
 */
error_code_t pus_services_TM_X_Y_write_saved_PIDValue(tm_handler_t *ptm_handler,
		uint16_t param_id,const data_pool_item_t *const p_pid_value);


#ifdef __cplusplus
}
#endif




#endif /* SERVICE_LIBRARIES_PUS_SERVICES_INCLUDE_PUS_SERVICES_AUX_PUS_SERVICES_UTILS_H_ */
