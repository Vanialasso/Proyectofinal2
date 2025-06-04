/*
 * icuasw_pus_service128.h
 *
 *  Created on: Dec 21, 2016
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


#ifndef PUBLIC__ICUASW_PUS_SERVICE128_H
#define PUBLIC__ICUASW_PUS_SERVICE128_H

#include "public/basic_types.h"

#include "public/pus_tc_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief executes a TC[128,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service128_exec_tc(tc_handler_t *ptc_handler);

#ifdef __cplusplus
}
#endif

#endif

