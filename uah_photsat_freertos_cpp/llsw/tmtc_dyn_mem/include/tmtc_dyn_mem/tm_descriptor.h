/*
 * tm_descritpor.h
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

#ifndef INCLUDE__TM_DESCRIPTOR__H_
#define INCLUDE__TM_DESCRIPTOR__H_

#include "public/basic_types.h"


/**
 * \brief Structure that stores a single TM descriptor. It contains a pointer
 *        to the allocated memory that stores the TM packet and the number of
 *        bytes that are actually in use by the TM packet.
 */
struct tm_mem_descriptor {

    uint8_t * p_tm_bytes;
    uint16_t tm_num_bytes;

};

typedef struct tm_mem_descriptor tm_mem_descriptor_t;


#endif /* INCLUDE__TM_DESCRIPTOR__H_ */
