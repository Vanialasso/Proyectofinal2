/*
 * tc_descritpor.h
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

#ifndef INCLUDE__TC_DESCRIPTOR__H_
#define INCLUDE__TC_DESCRIPTOR__H_

#include "public/basic_types.h"
/**
 * \brief Structure that stores a single TC descriptor. It contains a pointer
 *        to the allocated memory that stores the TC packet and the number of
 *        bytes that are actually in use by the TC packet.
 */
struct tc_mem_descriptor {

	//!pointer to the memory where the tc is stored
    uint8_t * p_tc_bytes;
    //!number of bytes of the tc
    uint16_t tc_num_bytes;

};

typedef struct tc_mem_descriptor tc_mem_descriptor_t;

#endif /* INCLUDE__TC_DESCRIPTOR__H_ */
