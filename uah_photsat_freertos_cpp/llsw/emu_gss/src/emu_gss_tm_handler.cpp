/*
 * emu_gss_tm_handler.cpp
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
#include "public/serialize.h"
#include "public/crc.h"
#include "public/ccsds_pus.h"


#define PUS_TM_TYPE_OFFSET 7
#define PUS_TM_SUBTYPE_OFFSET 8

#define PUS_TM_APPDATA_OFFSET (UAH_PUS_TM_DF_HEADER_SIZE + UAH_PUS_TM_TC_PACKET_HEADER_SIZE)

uint8_t GSSTMHandler::GetPUSType(const struct tm_mem_descriptor *pTMDescriptor) {

	uint8_t type = 0;
	if (pTMDescriptor->p_tm_bytes) {
		type = pTMDescriptor->p_tm_bytes[PUS_TM_TYPE_OFFSET];
	}
	return type;

}

uint8_t GSSTMHandler::GetPUSSubtype(
		const struct tm_mem_descriptor *pTMDescriptor) {

	uint8_t subtype = 0;
	if (pTMDescriptor->p_tm_bytes) {
		subtype = pTMDescriptor->p_tm_bytes[PUS_TM_SUBTYPE_OFFSET];
	}
	return subtype;

}

GSSTMHandler::GSSTMHandler(const uint8_t *pTMBytes) {

	mTMPacketHeader.packet_id = deserialize_uint16(pTMBytes);
	mTMPacketHeader.packet_seq_ctrl = deserialize_uint16(pTMBytes + 2);
	mTMPacketHeader.packet_length = deserialize_uint16(pTMBytes + 4);

	mDFHeader.version_obtstatus = pTMBytes[6];
	mDFHeader.type = pTMBytes[PUS_TM_TYPE_OFFSET];
	mDFHeader.subtype = pTMBytes[PUS_TM_SUBTYPE_OFFSET];

	mDFHeader.destinationID = deserialize_uint16(pTMBytes + 11);

	pmNextAppBytes = pTMBytes + PUS_TM_APPDATA_OFFSET;

}

uint8_t GSSTMHandler::GetNextUInt8AppDataField() {

	uint8_t data;
	data = *pmNextAppBytes;
	pmNextAppBytes++;
	return data;
}

uint16_t GSSTMHandler::GetNextUInt16AppDataField() {

	uint16_t data;
	data = deserialize_uint16(pmNextAppBytes);
	pmNextAppBytes += 2;
	return data;
}

uint32_t GSSTMHandler::GetNextUInt32AppDataField() {

	uint32_t data;
	data = deserialize_uint32(pmNextAppBytes);
	pmNextAppBytes += 4;
	return data;

}

uint64_t GSSTMHandler::GetNextUInt64AppDataField() {

	uint64_t data;
	data = deserialize_uint64(pmNextAppBytes);
	pmNextAppBytes += 8;
	return data;

}

int8_t GSSTMHandler::GetNextInt8AppDataField() {
	int8_t data;
	data = *pmNextAppBytes;
	pmNextAppBytes++;
	return data;

}

int16_t GSSTMHandler::GetNextInt16AppDataField() {

	int16_t data;
	data = deserialize_int16(pmNextAppBytes);
	pmNextAppBytes += 2;
	return data;

}

int32_t GSSTMHandler::GetNextInt32AppDataField() {

	int32_t data;
	data = deserialize_int32(pmNextAppBytes);
	pmNextAppBytes += 4;
	return data;

}

int64_t GSSTMHandler::GetNextInt64AppDataField() {

	int64_t data;
	data = deserialize_int64(pmNextAppBytes);
	pmNextAppBytes += 8;
	return data;
}
