/*
 * emu_gss_tc_program.cpp
 *
 *  Created on: Jan 9, 2024
 *
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
#include "public/serialize.h"
#include "public/crc.h"


EmuGSS_TCProgram *EmuGSS_TCProgram::sFirstTCProgram = NULL;
EmuGSS_TCProgram *EmuGSS_TCProgram::sCurrentTCProgram = NULL;
uint32_t EmuGSS_TCProgram::sTCSeqCounter = 0;

EmuGSS_TCProgram * EmuGSS_TCProgram::GetCurrentTCProgramed(){

	return EmuGSS_TCProgram::sCurrentTCProgram;

}

EmuGSS_TCProgram::EmuGSS_TCProgram(uint32_t obt_seconds, uint8_t type,
		uint8_t subtype, uint16_t appDataLength, const char *brief) {

	OBTSecs = obt_seconds;
	mType = type;
	mSubtype = subtype;
	mBrief = brief;
	mAppDataLength = appDataLength;
	pmAppData = NULL;
	mACKFlags=0x0F;
	pmNext=NULL;
}

bool EmuGSS_TCProgram::SetCurrentTCProgramed(uint32_t obt_seconds) {
	bool found = false;
	if (sFirstTCProgram) {
		if (sFirstTCProgram->OBTSecs <= obt_seconds) {
			sCurrentTCProgram = sFirstTCProgram;
			sFirstTCProgram = sFirstTCProgram->pmNext;
			found = true;
		}
	}
	return found;
}

void EmuGSS_TCProgram::NewProgram(EmuGSS_TCProgram *pTCProgram) {
	if (pTCProgram) {

		if (NULL == EmuGSS_TCProgram::sFirstTCProgram) {
			EmuGSS_TCProgram::sFirstTCProgram = pTCProgram;
			pTCProgram->pmNext = NULL;
		} else {
			bool found = false;
			EmuGSS_TCProgram *current;
			EmuGSS_TCProgram *prev = NULL;
			current = EmuGSS_TCProgram::sFirstTCProgram;
			while (!found) {
				if (current->OBTSecs > pTCProgram->OBTSecs) {
					if (prev) {
						prev->pmNext = pTCProgram;
					} else {
						EmuGSS_TCProgram::sFirstTCProgram = pTCProgram;
					}
					pTCProgram->pmNext = current;
					found = true;
				} else {
					prev = current;
					current = current->pmNext;
					if (NULL == current) {
						prev->pmNext = pTCProgram;
						pTCProgram->pmNext = NULL;
						found = true;
					}
				}
			}
		}
	}
}

void EmuGSS_TCProgram::BuildTC(tc_mem_descriptor_t &tc_descriptor) {

	if (tc_descriptor.p_tc_bytes) {

		BuildTCPackHeader(tc_descriptor);
		BuildTCDFHeader(tc_descriptor);
		pmAppData = tc_descriptor.p_tc_bytes + UAH_PUS_TC_APP_DATA_OFFSET;
		this->BuildTCAppData(tc_descriptor);

		uint16_t crc_offset = mAppDataLength + UAH_PUS_TC_APP_DATA_OFFSET;
		uint16_t crc_value = cal_crc_16(tc_descriptor.p_tc_bytes, crc_offset);
		serialize_uint16(crc_value, tc_descriptor.p_tc_bytes + crc_offset);

		tc_descriptor.tc_num_bytes=mAppDataLength + UAH_PUS_TC_APP_DATA_OFFSET
									+2;
	}

}

void EmuGSS_TCProgram::DisableAcceptACK(){

	mACKFlags&=0B11111110;

}
void EmuGSS_TCProgram::DisableExecStartACK(){

	mACKFlags&=0B11111101;
}
void EmuGSS_TCProgram::DisableExecProgACK(){

	mACKFlags&=0B11111011;
}
void EmuGSS_TCProgram::DisableExecCompletionACK(){

	mACKFlags&=0B11110111;
}


void EmuGSS_TCProgram::BuildTCPackHeader(tc_mem_descriptor_t &tc_descriptor) {

	if (tc_descriptor.p_tc_bytes) {
		uint16_t packet_id = ccsds_pus_tm_build_packet_id(UAH_APID);

		uint16_t packet_seq_ctrl = ccsds_pus_tm_build_packet_seq_ctrl(0x3,
				sTCSeqCounter);

		sTCSeqCounter++;

		uint16_t packet_length = UAH_PUS_TC_DF_HEADER_SIZE + 1
														+ mAppDataLength;

		serialize_uint16(packet_id, tc_descriptor.p_tc_bytes);

		// Serialize Packet Sequence Control from packet_seq_ctrl
		serialize_uint16(packet_seq_ctrl, tc_descriptor.p_tc_bytes + 2);

		// Serialize Packet Length from packet_length
		serialize_uint16(packet_length, tc_descriptor.p_tc_bytes + 4);

	}

}
void EmuGSS_TCProgram::BuildTCDFHeader(tc_mem_descriptor_t &tc_descriptor) {

	if (tc_descriptor.p_tc_bytes) {
		tc_descriptor.p_tc_bytes[6] = mACKFlags;
		tc_descriptor.p_tc_bytes[7] = mType;
		tc_descriptor.p_tc_bytes[8] = mSubtype;
		serialize_uint16(UAH_DESTINATION_ID,&tc_descriptor.p_tc_bytes[9]);
	}
}

void EmuGSS_TCProgram::SetNextUInt8(uint8_t data) {
	*pmAppData = data;
	pmAppData += 1;
}

void EmuGSS_TCProgram::SetNextUInt16(uint16_t data) {
	serialize_uint16(data, pmAppData);
	pmAppData += 2;
}

void EmuGSS_TCProgram::SetNextUInt32(uint32_t data) {
	serialize_uint32(data, pmAppData);
	pmAppData += 4;
}
void EmuGSS_TCProgram::SetNextUInt64(uint64_t data) {
	serialize_uint64(data, pmAppData);
	pmAppData += 8;
}

void EmuGSS_TCProgram::SetNextInt8(int8_t data) {
	*pmAppData = (uint8_t) data;
	pmAppData += 1;
}
void EmuGSS_TCProgram::SetNextInt16(int16_t data) {
	serialize_int16(data, pmAppData);
	pmAppData += 2;
}

void EmuGSS_TCProgram::SetNextInt32(int32_t data) {
	serialize_int32(data, pmAppData);
	pmAppData += 4;
}
void EmuGSS_TCProgram::SetNextInt64(int64_t data) {
	serialize_int64(data, pmAppData);
	pmAppData += 8;
}

void EmuGSS_TCProgram::SetNextFloat(float & data){
	uint32_t *p_aux;
	p_aux=(uint32_t *)(&data);
	serialize_uint32(*p_aux, pmAppData);
		pmAppData += 4;
}
