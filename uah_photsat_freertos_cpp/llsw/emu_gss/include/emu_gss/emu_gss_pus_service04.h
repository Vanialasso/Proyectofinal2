/*
 * emu_gss_pus_service4.h
 *
 *  Created on: Nov 21, 2024
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

#ifndef LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_PUS_SERVICE04_H_
#define LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_PUS_SERVICE04_H_

class EmuGSS_TCProgram4_1:public EmuGSS_TCProgram{

  void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);


public:

  EmuGSS_TCProgram4_1(uint32_t uniTime2YK,
                 const char * brief);


};

class EmuGSS_TCProgram4_6:public EmuGSS_TCProgram{

	uint16_t mPID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram4_6(uint32_t uniTime2YK,
                 const char * brief,uint16_t PID);


};

class EmuGSS_TCProgram4_7:public EmuGSS_TCProgram{

	uint16_t mPID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram4_7(uint32_t uniTime2YK,
                 const char * brief,uint16_t PID);


};

void EmuGSS_ShowServ4TM(const struct tm_mem_descriptor *pTMDescriptor);


class GSSServ4TMHandler:public GSSTMHandler{

	friend void EmuGSS_ShowServ4TM(const struct tm_mem_descriptor * );

protected:

	GSSServ4TMHandler(const uint8_t * pTMBytes);

	void ShowTM();

	void ShowTM_4_2();
	void ShowTM_4_2_PIDValue();
};



#endif /* LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_PUS_SERVICE04_H_ */
