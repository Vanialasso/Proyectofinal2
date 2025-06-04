/*
 * device_drv.h
 *
 *  Created on: Nov 14, 2024
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

#ifndef LLSW_DEVICE_DRV_INCLUDE_DEVICE_DRV_H_
#define LLSW_DEVICE_DRV_INCLUDE_DEVICE_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "public/config.h"
#include "public/basic_types.h"


/**
 * \brief execute a device on/off command
 * \param on_off_address address asigned to the on/off command
 * \return error 0 if succeed, 1 if command fails
 */
error_code_t device_drv_on_off(uint32_t on_off_address);

/**
 * \brief check if a device on/off address is valid
 * \param on_off_address address asigned to the on/off command
 * \return true if this address is valid
 */
bool_t device_drv_is_on_off_address_valid(uint32_t on_off_address);


#ifdef __cplusplus
}
#endif



#endif /* LLSW_DEVICE_DRV_INCLUDE_DEVICE_DRV_H_ */
