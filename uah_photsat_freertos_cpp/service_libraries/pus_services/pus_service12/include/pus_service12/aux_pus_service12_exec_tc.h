/*
 * aux_pus_services12_exec_tc.h
 *
 *  Created on: Oct 28, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_AUX_PUS_SERVICE12_EXEC_TC_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_AUX_PUS_SERVICE12_EXEC_TC_H_

#include "public/config.h"
#include <public/pus_tc_handler.h>


/**
 * \brief executes a TC[12,1] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service12_exec_TC_12_1(tc_handler_t *ptc_handler);

/**
 * \brief executes a TC[12,2] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service12_exec_TC_12_2(tc_handler_t *ptc_handler);

/**
 * \brief executes a TC[12,5] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service12_exec_TC_12_5(tc_handler_t *ptc_handler);

/**
 * \brief executes a TC[12,6] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service12_exec_TC_12_6(tc_handler_t *ptc_handler);



#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE12_INCLUDE_AUX_PUS_SERVICES12_EXEC_TC_H_ */
