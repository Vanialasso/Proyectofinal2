/*
 * aux_pus_service5_utils.h
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE05_INCLUDE_PUS_SERVICE5_AUX_PUS_SERVICE5_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE05_INCLUDE_PUS_SERVICE5_AUX_PUS_SERVICE5_UTILS_H_

#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_handler.h"
#include "public/pus_tm_handler.h"






/**
 * \brief calculate the TM[5,X] subtype that corresponds to a valid EvID
 * \param EvID Event Identifier
 * \return X where EvID corresponds to TM[5,X]
 */
uint8_t pus_service5_cal_EvID_TM_5_X_subtype(uint16_t validEvID);

/**
 * \brief get the enable_config index
 * \param EvID Event Identifier
 * \return the index in the enable config
 */
uint8_t pus_service5_get_EvID_enable_config_index(uint16_t EvID);

/**
 * \brief check the enable_config flag value
 * \param EvID Event Identifier
 * \return return the value of flag of the enable config
 */
bool_t pus_service5_check_EvID_enable_flag(uint16_t EvID);


/**
 * \brief set the enable_config flag value
 * \param EvID Event Identifier
 * \param flag_value the flag value
 */
void pus_service5_set_EvID_enable_flag(uint16_t EvID,bool_t flag_value);



#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE05_INCLUDE_PUS_SERVICE5_AUX_PUS_SERVICE5_UTILS_H_ */
