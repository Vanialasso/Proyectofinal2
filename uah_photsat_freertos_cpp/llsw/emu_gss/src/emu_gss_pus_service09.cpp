/*
 * emu_sc_pus_service9.cpp
 *
 *  Created on: Jan 13, 2017
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

#include "public/emu_gss_v1.h"

#define TC_9_129_APPDATA_LENGTH 4

EmuGSS_TCProgram9_129::EmuGSS_TCProgram9_129(uint32_t uniTime2YK,
                         const char * brief,
                         uint32_t nextUniTime2YK)
                        :EmuGSS_TCProgram(uniTime2YK,9,129,
                        		TC_9_129_APPDATA_LENGTH,brief){

	mNextUniTime2YK=nextUniTime2YK;
    NewProgram(this);
}

void EmuGSS_TCProgram9_129::BuildTCAppData(tc_mem_descriptor_t &tc_descriptor){

	SetNextUInt32(mNextUniTime2YK);

}
