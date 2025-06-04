/*
 * aus_pus_service5_utils.c
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

#include "../../pus_service05/include/pus_service5/aux_pus_service5_utils.h"

#include "../../pus_service05/include/public/pus_service5.h"

//Enable Config, the events defined are enabled by default
uint32_t EvIDEnableConfig[4] =
		{ 0x00000007, 0x00000000, 0x00000000, 0x00000007 };


uint8_t pus_service5_cal_EvID_TM_5_X_subtype(uint16_t validEvID) {

	return (validEvID >> 12);

}

bool_t pus_service5_check_EvID_enable_flag(uint16_t EvID) {

	bool_t is_enabled = false;
	ev_type_t event_type = pus_service5_get_EvID_type(EvID);
	uint8_t index = 4; //Not valid index.

	uint8_t num_of_events_of_type;

	switch (event_type) {
	case (EvTypeInformative):
		index = 0;
		num_of_events_of_type = PUS_5_InformativeEvents;
		break;
	case (EvTypeLowSevAnomaly):
		index = 1;
		num_of_events_of_type = PUS_5_LowSeverityAnomalyEvents;
		break;
	case (EvTypeMediumSevAnomaly):
		index = 2;
		num_of_events_of_type = PUS_5_MediumSeverityAnomalyEvents;
		break;
	case (EvTypeHighSevAnomaly):
		index = 3;
		num_of_events_of_type = PUS_5_HighSeverityAnomalyEvents;
		break;
	default:
		index = 4;
		num_of_events_of_type = 0;
	}

	if (index < 4) {
		uint8_t aux_EvID_offset = (EvID & 0xFFF);

		//offset = aux_EvID_offset-1;
		//offset must be >0 and < number of events of that type

		if ((aux_EvID_offset > 0)
				&& (aux_EvID_offset <= num_of_events_of_type)) {

			is_enabled = (EvIDEnableConfig[index] >> (aux_EvID_offset-1))
					& 0x01;
		}
	}

	return is_enabled;
}

void pus_service5_set_EvID_enable_flag(uint16_t EvID, bool_t enable_value) {

	ev_type_t event_type = pus_service5_get_EvID_type(EvID);
	uint8_t index = 4; //Not valid index.

	uint8_t num_of_events_of_type;

	switch (event_type) {
	case (EvTypeInformative):
		index = 0;
		num_of_events_of_type = PUS_5_InformativeEvents;
		break;
	case (EvTypeLowSevAnomaly):
		index = 1;
		num_of_events_of_type = PUS_5_LowSeverityAnomalyEvents;
		break;
	case (EvTypeMediumSevAnomaly):
		index = 2;
		num_of_events_of_type = PUS_5_MediumSeverityAnomalyEvents;
		break;
	case (EvTypeHighSevAnomaly):
		index = 3;
		num_of_events_of_type = PUS_5_HighSeverityAnomalyEvents;
		break;
	default:
		index = 4;
		num_of_events_of_type = 0;
	}

	//offset + 1 must be <= event_number
	if (index < 4) {
		uint8_t aux_EvID_offset = (EvID & 0xFFF);

		//offset = aux_EvID_offset-1;
		//offset must be >0 and < number of events of that type

		if ((aux_EvID_offset > 0)
				&& (aux_EvID_offset <= num_of_events_of_type)) {

			uint8_t mask = (0x01 << (aux_EvID_offset - 1));
			if (enable_value) {
				EvIDEnableConfig[index] |= mask;
			} else {
				EvIDEnableConfig[index] &= ~(mask);
			}
		}
	}

}

