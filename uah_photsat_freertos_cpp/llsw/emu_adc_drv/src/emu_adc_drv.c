/*
 * emu_adc_drv.h
 *
 *  Created on: Apr 27, 2024
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

#include <public/pus_sys_data_pool.h>
#include <stdlib.h>
#include "public/adc_drv.h"

#define EMU_PARAM_NUM 5

//0 is -10  4095 is + 10
static uint16_t emu_sys_data_pool_parameters[EMU_PARAM_NUM] = { 2375, 2621,
		2723, 3072, 2048 };

/**
 * \brief Adds noise to a given parameter value, ensuring the noisy value
 *        remains within the same multiple of 10 as the original value.
 *
 * The function calculates the lower and upper limits of the range for the
 * noisy value, based on the original value. It generates a random noise value
 * between -5 and 5 (inclusive) and adds it to the original value. The noisy
 * value is then clamped to the calculated range to ensure it remains within
 * the same multiple of 10 as the original value.
 *
 * @param value The original uint8_t value to which noise will be added.
 * @return The noisy value after adding the noise and clamping it within the
 *         defined range.
 */
uint16_t get_noise() {

	// Generate a random noise value between -25 and 25 (inclusive)
	int16_t noise_value = (rand() % 51) - 25;

	// Return the noisy value as a uint8_t integer
	return noise_value;
}

uint16_t sample_adc_channel(uint8_t channel_id) {

	uint16_t emu_sample_value = 0;

	if (channel_id < EMU_PARAM_NUM) {

		emu_sample_value = emu_sys_data_pool_parameters[channel_id];

		emu_sample_value += get_noise();

	}

	return emu_sample_value;

}

//Sample ADC channel and return value
void sample_sys_data_pool_params() {

	for (int i = 0; i < EMU_PARAM_NUM; i++) {

		sys_data_pool_set_uint32_param(i, sample_adc_channel(i));

	}

}

void init_adc_channel() {

	for (int i = 0; i < EMU_PARAM_NUM; i++) {

		sys_data_pool_set_uint32_param(i, sample_adc_channel(i));

	}

	/*
	for (int i = EMU_PARAM_NUM; i < SYS_DATA_POOL_UINT32_PARAMS; i++) {

		sys_data_pool_set_uint32_param(i, i);

	}

	for (int i = SYS_DATA_POOL_UINT32_PARAMS;
			i < SYS_DATA_POOL_UINT32_PARAMS + SYS_DATA_POOL_UINT8_PARAMS; i++) {

		sys_data_pool_set_uint8_param(i, i);

	}*/


}
