/*
 * emu_sc_pus_service3.h
 *
 *  Created on: Jan 16, 2024
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

#ifndef EMU_SC_PUS_SERVICE20_H_
#define EMU_SC_PUS_SERVICE20_H_

class EmuGSS_TCProgram20_1:public EmuGSS_TCProgram{

	uint16_t mPID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram20_1(uint32_t uniTime2YK,
                 const char * brief,uint16_t PID);


};

class EmuGSS_TCProgram20_3_uint8:public EmuGSS_TCProgram{

	uint16_t mPID;
	uint8_t mValue;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram20_3_uint8(uint32_t uniTime2YK,
                 const char * brief,uint16_t PID, uint8_t value);


};

class EmuGSS_TCProgram20_3_uint32:public EmuGSS_TCProgram{

	uint16_t mPID;
	uint32_t mValue;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram20_3_uint32(uint32_t uniTime2YK,
                 const char * brief,uint16_t PID, uint32_t value);


};



void EmuGSS_ShowServ20TM(const struct tm_mem_descriptor *pTMDescriptor);


class GSSServ20TMHandler:public GSSTMHandler{

	friend void EmuGSS_ShowServ20TM(const struct tm_mem_descriptor * );

protected:

	GSSServ20TMHandler(const uint8_t * pTMBytes);

	void ShowTM();

	void ShowTM_20_2();


};



#endif /* EMU_SC_PUS_SERVICE9_H_ */
