/*
 * emu_sc_pus_service17.h
 *
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

#ifndef EMU_SC_PUS_SERVICE17_H_
#define EMU_SC_PUS_SERVICE17_H_

class EmuGSS_TCProgram17_1:public EmuGSS_TCProgram{

  void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);


public:

  EmuGSS_TCProgram17_1(uint32_t uniTime2YK,
                 const char * brief);


};

class EmuGSS_TCProgram17_1_DISABLE_1_7:public EmuGSS_TCProgram{

  void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);


public:

  EmuGSS_TCProgram17_1_DISABLE_1_7(uint32_t uniTime2YK,
                 const char * brief);


};


class EmuGSS_TCProgram17_3:public EmuGSS_TCProgram{

  virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

  uint16_t mAPID;

public:

  EmuGSS_TCProgram17_3(uint32_t uniTime2YK,const char * brief,uint16_t APID);

};


#endif /* EMU_SC_PUS_SERVICE17_H_ */
