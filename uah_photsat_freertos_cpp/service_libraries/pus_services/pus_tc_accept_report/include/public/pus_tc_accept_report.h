/*
 * tc_accept_report.h
 *
 *  Created on: Oct 9, 2024
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

#ifndef SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_TC_ACCEPT_REPORT_INCLUDE_ASW_TC_ACCEPT_REPORT_H_
#define SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_TC_ACCEPT_REPORT_INCLUDE_ASW_TC_ACCEPT_REPORT_H_

#include "public/basic_types.h"
#include "public/tmtc_dyn_mem.h"

#ifdef __cplusplus
extern "C" {
#endif

enum TTCAcceptationStatus {
	TCAcceptationNotProcessed,
	TCAcceptationOK,
	TCAcceptationCRCError,
	TCAcceptationAPIDError,
	TCAcceptationSourceIDError,
	TCAcceptationInObservationModeError,
	TCAcceptationTypeError,
	TCAcceptationSubTypeError
};

struct tc_accept_report {

	//!Calculated packet err_ctrl;
	uint16_t cal_packet_err_ctrl;

	//!Attribute that defines the acceptation error;
	enum TTCAcceptationStatus accept_status;

};

typedef struct tc_accept_report tc_accept_report_t;

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_ASW_PUS_SERVICES_ASW_TC_ACCEPT_REPORT_INCLUDE_ASW_TC_ACCEPT_REPORT_H_ */
