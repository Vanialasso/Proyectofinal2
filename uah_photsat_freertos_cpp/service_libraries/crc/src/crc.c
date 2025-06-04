/*
 * crc.h
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

#include "public/crc.h"

uint16_t cal_crc_16(uint8_t data[], uint16_t nbytes) {

    uint16_t crc_value = 0xFFFF;

    for (uint16_t i = 0; i < nbytes; i++ ) {

        crc_value = crc_value ^ (data[i] << 8);

        for (uint8_t j = 0; j < 8; j++) {

            if ((crc_value & 0x8000) != 0) {
                crc_value = (crc_value << 1) ^ 0x1021;
            } else {
                crc_value = crc_value << 1;
            }
        }
    }

    return crc_value;
}
