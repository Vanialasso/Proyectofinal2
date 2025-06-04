/*
 * aux_pus_service129_observ_mng.h
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE129_INCLUDE_PUS_SERVICE1_AUX_PUS_SERVICE129_OBS_MNG_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE129_INCLUDE_PUS_SERVICE1_AUX_PUS_SERVICE129_OBS_MNG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <public/pus_tm_handler.h>
#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_handler.h"

/**
 * \brief Set Ks PID Controller
 * \param Kp Kp of PID Controller
 * \param Ki Ki of PID Controller
 * \param Kd Kd of PID Controller
 *
 */
void pus_service129_set_Kp_Ki_Kd(float kp, float ki, float kd);

/**
 * \brief Set Attitude Target
 * \param theta_pitch Thetha of Pitch angle
 * \param theta_yaw Thetha of Yaw angle
 *
 */
void pus_service129_set_attitude_target(int16_t theta_pitch, int16_t theta_yaw);

/**
 * \brief Set Number of Images
 * \param num_of_images
 */

void pus_service129_set_num_of_images(uint8_t num_of_images);

/**
 * \brief Get Pending Number of Images
 * \return pending num of images
 */

uint8_t pus_service129_get_pending_num_of_images();

/**
 * \brief Manage observation mode entry
 */

void pus_service129_observ_mode_entry();

/**
 * \brief Manage observation mode exit
 */

void pus_service129_observ_mode_exit();


/**
 * \brief Tx image tm
 *
 */

void pus_service129_mng_image_adquisition();


/**
 * \brief Apply Ctrl Algorithm
 *
 */
void pus_service129_apply_ctrl_algorithm();

/**
 * \brief Cal and apply torques
 *
 */
void pus_service129_update_system_data_pool();


/**
 * \brief Attitude error under limits
 *\return true if attitude error is under limits
 */
bool_t pus_service129_attitude_error_under_limits();

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE129_INCLUDE_PUS_SERVICE1_AUX_PUS_SERVICE129_OBS_MNG_H_ */
