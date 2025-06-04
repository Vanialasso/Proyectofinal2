/*
 * asw_pus_service3.h
 *
 *  Created on: Oct 21, 2024
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


#ifndef SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_PUS_SERVICE3_INCLUDE_ASW_PUS_SERVICE3_H_
#define SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_PUS_SERVICE3_INCLUDE_ASW_PUS_SERVICE3_H_

#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_handler.h"
#include "public/pus_tm_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PUS_SERVICE3_MAX_NUM_OF_SIDS 16

#define PUS_SERVICE3_MAX_NUM_OF_PID_PER_SID 12

enum SID_config_status{SIDEnabled, SIDDisabled, SIDConfigUnused};

struct HK_config{
  enum SID_config_status  status;
  uint16_t SID;
  uint8_t interval;
  uint8_t interval_ctrl;
  uint8_t num_of_params;
  uint16_t param_IDs[PUS_SERVICE3_MAX_NUM_OF_PID_PER_SID];
};

typedef struct HK_config HK_config_t;

/**
 * \brief Do HK, managing the generation of TM[3,25]
 */
void pus_service3_do_HK();

/**
 * \brief executes a TC[3,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service3_exec_tc(tc_handler_t *ptc_handler);

#ifdef __cplusplus
}
#endif


#endif /* SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_PUS_SERVICE3_INCLUDE_ASW_PUS_SERVICE3_H_ */
