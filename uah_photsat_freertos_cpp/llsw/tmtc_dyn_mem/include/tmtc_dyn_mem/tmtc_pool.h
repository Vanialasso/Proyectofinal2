/*
 * tmtc_pool.h
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


#ifndef INCLUDE__TMTC_POOL__H_
#define INCLUDE__TMTC_POOL__H_

#include "public/basic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initializes the TM/TC pool static structures.
 */
void init_tmtc_pool();

/**
 * \brief Allocates a new block from the memory pool.
 *
 * @return The address of the allocated block or NULL if there were no free
 *         blocks in the pool.
 */
uint8_t * tmtc_pool_alloc();

/**
 * \brief Frees a previously allocated block of the memory pool.
 *
 * @param p_block the address of the block to deallocate.
 */
void tmtc_pool_free(uint8_t * p_block);




#ifdef __cplusplus
}
#endif

#endif /* INCLUDE__TMTC_POOL__H_ */
