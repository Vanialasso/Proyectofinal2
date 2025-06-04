/*
 * serialize.h
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

#ifndef INCLUDE__SERIALIZE_H_
#define INCLUDE__SERIALIZE_H_

#include "public/config.h"
#include "public/basic_types.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief Deserializes a 16-bit integer
 *
 * \param data_bytes vector containing the MSB and LSB bytes of the word
 *
 * \return the deserialized 16-bit integer
 */
uint16_t deserialize_int16(const uint8_t data_bytes[]);


/**
 * \brief Deserializes a 16-bit word
 *
 * \param data_bytes vector containing the MSB and LSB bytes of the word
 *
 * \return the deserialized 16-bit word
 */
uint16_t deserialize_uint16(const uint8_t data_bytes[]);

/**
 * \brief Deserializes a 32-bit integer
 *
 * \param data_bytes vector containing the 4 bytes of the word
 *
 * \return the deserialized 32-bit integer
 */
int32_t deserialize_int32(const uint8_t data_bytes[]);

/**
 * \brief Deserializes a 32-bit word
 *
 * \param data_bytes vector containing the 4 bytes of the word
 *
 * \return the deserialized 32-bit word
 */
uint32_t deserialize_uint32(const uint8_t data_bytes[]);

/**
 * \brief Deserializes a 64-bit integer
 *
 * \param data_bytes vector containing the 8 bytes of the word
 *
 * \return the deserialized 64-bit integer
 */
int64_t deserialize_int64(const uint8_t data_bytes[]);

/**
 * \brief Deserializes a 64-bit word
 *
 * \param data_bytes vector containing the 8 bytes of the word
 *
 * \return the deserialized 64-bit word
 */
uint64_t deserialize_uint64(const uint8_t data_bytes[]);

/**
 * \brief Deserializes a float word
 *
 * \param data_bytes vector containing the 4 bytes of the float
 *
 * \return the deserialized 32-bit float
 */
float deserialize_float(const uint8_t data_bytes[]);

/**
 * \brief Serializes a 16-bit integer
 *
 * \param var the 16-bit integer to serialize
 * \param data_bytes vector that will store the serialized 16-bit integer
 *
 */
void serialize_int16(int16_t var, uint8_t data_bytes[]);


/**
 * \brief Serializes a 16-bit word
 *
 * \param var the 16-bit word to serialize
 * \param data_bytes vector that will store the serialized 16-bit word
 *
 */
void serialize_uint16(uint16_t var, uint8_t data_bytes[]);

/**
 * \brief Serializes a 32-bit integer
 *
 * \param var the 32-bit integer to serialize
 * \param data_bytes vector that will store the serialized 32-bit integer
 *
 */
void serialize_int32(int32_t var, uint8_t data_bytes[]);


/**
 * \brief Serializes a 32-bit word
 *
 * \param var the 32-bit word to serialize
 * \param data_bytes vector that will store the serialized 32-bit word
 *
 */
void serialize_uint32(uint32_t var, uint8_t data_bytes[]);

/**
 * \brief Serializes a 64-bit integer
 *
 * \param var the 64-bit integer to serialize
 * \param data_bytes vector that will store the serialized 64-bit integer
 *
 */
void serialize_int64(int64_t var, uint8_t data_bytes[]);


/**
 * \brief Serializes a 64-bit word
 *
 * \param var the 64-bit word to serialize
 * \param data_bytes vector that will store the serialized 32-bit word
 *
 */
void serialize_uint64(uint64_t var, uint8_t data_bytes[]);


/**
 * \brief Serializes a 32-bit float
 *
 * \param var the 32-bit float to serialize
 * \param data_bytes vector that will store the serialized 32-bit float
 *
 */
void serialize_float(float var, uint8_t data_bytes[]);

#ifdef __cplusplus
}
#endif


#endif /* INCLUDE__SERIALIZE_H_ */
