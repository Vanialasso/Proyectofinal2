/*
 * emu_gss_pus_service1.h
 *
 *  Created on: Jan 4, 2024
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

#ifndef LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_PUS_SERVICE01_H_
#define LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_PUS_SERVICE01_H_

#include "public/emu_gss_v1.h"


void EmuGSS_ShowServ1TM(const struct tm_mem_descriptor *pTMDescriptor);


class GSSServ1TMHandler:public GSSTMHandler{

	friend void EmuGSS_ShowServ1TM(const struct tm_mem_descriptor * );

protected:

	GSSServ1TMHandler(const uint8_t * pTMBytes);

	void ShowTM();

	void ShowTM_1_2();

	void ShowTM_1_4();

	void ShowTM_1_8();


};




#endif /* LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_PUS_SERVICE01_H_ */
