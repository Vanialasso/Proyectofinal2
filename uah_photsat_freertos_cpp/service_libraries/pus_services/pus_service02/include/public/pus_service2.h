/*
 * pus_service2.h
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



#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE02_INCLUDE_PUBLIC_PUS_SERVICE2_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE02_INCLUDE_PUBLIC_PUS_SERVICE2_H_



#include "public/basic_types.h"

#include "public/pus_tc_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief executes a TC[2,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service2_exec_tc(tc_handler_t *ptc_handler);

#ifdef __cplusplus
}
#endif


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE02_INCLUDE_PUBLIC_PUS_SERVICE2_H_ */
