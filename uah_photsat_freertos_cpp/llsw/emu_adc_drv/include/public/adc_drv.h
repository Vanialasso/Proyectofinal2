/*
 * adc_drv.h
 *
 *  Created on: May 5, 2024
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

#ifndef LLSW_EMU_ADC_DRV_INCLUDE_ADC_DRV_H_
#define LLSW_EMU_ADC_DRV_INCLUDE_ADC_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "public/config.h"
#include "public/basic_types.h"

//Get sample ADC channel and return value
uint16_t sample_adc_channel(uint8_t channel_id) ;

//Sample system data pool parameters
void sample_sys_data_pool_params();

//Init ADC channel
void init_adc_channel();

#ifdef __cplusplus
}
#endif

#endif

