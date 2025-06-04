/*
 * emu_sc_pus_service3.h
 *
 *  Created on: Jan 16, 2017
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

#ifndef EMU_SC_PUS_SERVICE3_H_
#define EMU_SC_PUS_SERVICE3_H_

class EmuGSS_TCProgram3_5:public EmuGSS_TCProgram{

	uint16_t mSID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram3_5(uint32_t uniTime2YK,
                 const char * brief,uint16_t SID);


};

class EmuGSS_TCProgram3_6:public EmuGSS_TCProgram{

	uint16_t mSID;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram3_6(uint32_t uniTime2YK,
                 const char * brief,uint16_t SID);


};

class EmuGSS_TCProgram3_31:public EmuGSS_TCProgram{

	uint16_t mSID;
	uint8_t mCollectInterval;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram3_31(uint32_t uniTime2YK,
                 const char * brief,uint16_t SID, uint8_t collectInterval);


};


void EmuGSS_ShowServ3TM(const struct tm_mem_descriptor *pTMDescriptor);


class GSSServ3TMHandler:public GSSTMHandler{

	friend void EmuGSS_ShowServ3TM(const struct tm_mem_descriptor * );

protected:

	GSSServ3TMHandler(const uint8_t * pTMBytes);

	void ShowTM();

	void ShowTM_3_25();
	void ShowTM_3_25_SID(uint16_t SID);
};



#endif /* EMU_SC_PUS_SERVICE9_H_ */
