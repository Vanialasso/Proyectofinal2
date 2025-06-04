/*
 * tc_rate_ctrl.h
 *
 *  Created on: Oct 26, 2024
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

#ifndef PUBLIC_TC_RATE_CTRL_H
#define PUBLIC_TC_RATE_CTRL_H


#include "public/config.h"
#include "public/basic_types.h"


/**
 * \brief	TC Rate Ctrl().
 */
void RxTC_RateCtrl();


/**
 * \brief	return true if Max Number of TC per seconds has been
 * 			exceeded
 */
bool RxTC_TCRateExceeded();


#endif // PUBLIC__CONSOLE_DRV_IFACE_V1_H
