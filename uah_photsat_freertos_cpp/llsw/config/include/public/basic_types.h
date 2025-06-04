/*
 * basic_types.h
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

#ifndef LEON3_TYPES_H
#define LEON3_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifndef NULL
	#define NULL  0
#endif


#ifndef __cplusplus

	typedef unsigned char 		bool_t;

	#define true 	1
	#define false 	0

#else

	typedef bool 		bool_t;

#endif


typedef unsigned char 		byte_t;
typedef unsigned short int	word16_t;
typedef unsigned int		word32_t;
typedef unsigned long int	word64_t;

typedef int error_code_t ;



#endif /* LEON3_TYPES_H_ */
