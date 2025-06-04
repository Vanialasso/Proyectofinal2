/*
 * serialize.c
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

#include "public/serialize.h"

uint16_t deserialize_int16(const uint8_t data_bytes[]) {

	union {
		int16_t data;
		uint8_t bytes[2];
	} serializer;

	for (int i = 0; i < 2; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[1-i]=data_bytes[i];
#endif

	}

	return serializer.data;

}

uint16_t deserialize_uint16(const uint8_t data_bytes[]) {

	union {
		uint16_t data;
		uint8_t bytes[2];
	} serializer;

	for (int i = 0; i < 2; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[1-i]=data_bytes[i];
#endif

	}

	return serializer.data;

}

int32_t deserialize_int32(const uint8_t data_bytes[]) {

	union {
		int32_t data;
		uint8_t bytes[4];
	} serializer;

	for (int i = 0; i < 4; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[3-i]=data_bytes[i];
#endif

	}

	return serializer.data;

}

uint32_t deserialize_uint32(const uint8_t data_bytes[]) {

	union {
		uint32_t data;
		uint8_t bytes[4];
	} serializer;

	for (int i = 0; i < 4; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[3-i]=data_bytes[i];
#endif

	}

	return serializer.data;

}

int64_t deserialize_int64(const uint8_t data_bytes[]) {

	union {
		int64_t data;
		uint8_t bytes[8];
	} serializer;

	for (int i = 0; i < 8; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[7-i]=data_bytes[i];
#endif

	}

	return serializer.data;

}

uint64_t deserialize_uint64(const uint8_t data_bytes[]) {

	union {
		uint64_t data;
		uint8_t bytes[8];
	} serializer;

	for (int i = 0; i < 8; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[7-i]=data_bytes[i];
#endif

	}

	return serializer.data;

}

float deserialize_float(const uint8_t data_bytes[]){

	union {
		float data;
		uint8_t bytes[4];
	} serializer;

	for (int i = 0; i < 4; i++) {

#ifdef BIG_ENDIAN
		serializer.bytes[i] = data_bytes[i];
#else
		serializer.bytes[3-i]=data_bytes[i];
#endif

	}

	return serializer.data;


}


void serialize_int16(int16_t var, uint8_t data_bytes[]) {

	union {
		int16_t data;
		uint8_t bytes[2];
	} serializer;

	serializer.data = var;

	for (int i = 0; i < 2; i++)

#ifdef BIG_ENDIAN

		data_bytes[i] = serializer.bytes[i];

#else



		data_bytes[i] = serializer.bytes[1 - i];

#endif

}

void serialize_uint16(uint16_t var, uint8_t data_bytes[]) {

	union {
		uint16_t data;
		uint8_t bytes[2];
	} serializer;

	serializer.data = var;

	for (int i = 0; i < 2; i++)

#ifdef BIG_ENDIAN
		data_bytes[i] = serializer.bytes[i];
#else
		data_bytes[i] = serializer.bytes[1 - i];
#endif

}

void serialize_int32(int32_t var, uint8_t data_bytes[]) {

	union {
		int32_t data;
		uint8_t bytes[4];
	} serializer;

	serializer.data = var;

	for (int i = 0; i < 4; i++)

#ifdef BIG_ENDIAN
		data_bytes[i] = serializer.bytes[i];

#else
		data_bytes[i] = serializer.bytes[3 - i];

#endif

}

void serialize_uint32(uint32_t var, uint8_t data_bytes[]) {
	union {
		uint32_t data;
		uint8_t bytes[4];
	} serializer;

	serializer.data = var;

	for (int i = 0; i < 4; i++)

#ifdef BIG_ENDIAN
		data_bytes[i] = serializer.bytes[i];
#else
		data_bytes[i] = serializer.bytes[3 - i];

#endif

}

void serialize_int64(int64_t var, uint8_t data_bytes[]) {

	union {
	    int64_t data;
	    uint8_t bytes[8];
	} serializer;

	serializer.data=var;

	for (int i = 0; i < 8; i++)

#ifdef BIG_ENDIAN
		data_bytes[i] = serializer.bytes[i];
#else

		data_bytes[i] = serializer.bytes[7 - i];

#endif

}

void serialize_uint64(uint64_t var, uint8_t data_bytes[]) {

	union {
	    uint64_t data;
	    uint8_t bytes[8];
	} serializer;

	serializer.data=var;

	for (int i = 0; i < 8; i++)

#ifdef BIG_ENDIAN
		data_bytes[i] = serializer.bytes[i];
#else

		data_bytes[i] = serializer.bytes[7 - i];

#endif

}

void serialize_float(float var, uint8_t data_bytes[]) {
	union {
		float data;
		uint8_t bytes[4];
	} serializer;

	serializer.data = var;

	for (int i = 0; i < 4; i++)

#ifdef BIG_ENDIAN
		data_bytes[i] = serializer.bytes[i];
#else
		data_bytes[i] = serializer.bytes[3 - i];

#endif

}
