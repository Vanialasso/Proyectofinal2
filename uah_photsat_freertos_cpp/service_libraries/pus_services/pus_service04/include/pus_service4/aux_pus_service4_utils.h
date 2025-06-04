/*
 * aux_pus_service4_utils.h
 *
 *  Created on: Nov 20, 2024
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


#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE04_INCLUDE_PUS_SERVICE4_AUX_PUS_SERVICE4_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE04_INCLUDE_PUS_SERVICE4_AUX_PUS_SERVICE4_UTILS_H_



#include "public/config.h"
#include "public/basic_types.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/pus_tm_handler.h"
#include <public/pus_sys_data_pool.h>
#include "../../../pus_service01/include/public/pus_service1.h"
#include "../../../pus_service04/include/public/pus_service4.h"


#ifdef __cplusplus
extern "C" {
#endif

struct stats_def{
	data_pool_item_t min;
	uint32_t min_obt;
	data_pool_item_t max;
	uint32_t max_obt;
	data_pool_item_t mean_value;
	uint32_t samples;
};

typedef struct stats_def stats_def_t;

struct stats_config {
	bool_t  defined[PUS_SERVICE4_MAX_NUM_OF_STATS];
	uint16_t PID[PUS_SERVICE4_MAX_NUM_OF_STATS];
	uint32_t start_time[PUS_SERVICE4_MAX_NUM_OF_STATS];
};


typedef struct stats_config stats_config_t;

extern stats_def_t ParamStats[PUS_SERVICE4_MAX_NUM_OF_STATS];

extern stats_config_t StatsConfig;




/**
 * \brief add the PID statistitcs
 * \param param_id parameter identifier
 * \return 0 only if statistic can be enabled
 */
error_code_t pus_service4_add_PID_stats(uint16_t param_id);

/**
 * \brief delete the PID statistitcs
 * \param param_id parameter identifier
 * \return 0 only if statistic can be deleted
 */
error_code_t pus_service4_delete_PID_stats(uint16_t param_id);

/**
 * \brief delete all statistitcs
 * \return 0 only if statistic can be deleted
 */
error_code_t pus_service4_delete_all_stats();

/**
 * \brief return true if stats for this index is defined and PID is valid
 * \param stat_index a valid index
 * \return 0 only if valid index stats is defined and PID valid, 1 if not
 */
bool_t pus_service4_is_stats_index_defined(uint16_t stat_index);

/**
 * \brief return true if stat for this param id is defined
 * \param stat_index pointer to the a valid index
 * \return 0 only if SID is valid, 1 if not
 */
bool_t pus_service4_is_PID_stats_defined(uint16_t param_id);


/**
 * \brief return the total of stats defined
 * \return 0 only no stats is defined
 */
uint8_t pus_service4_total_stats_defined();


/**
 * \brief get the defined PID index in the HKConfig structure
 * \param struct_id housekeeping structure identifier
 * \param p_validindex pointer to the a valid index
 * \return 0 only if pid_index exists and is defined
 */
error_code_t  pus_service4_get_PID_defined_stats_index(uint16_t param_id,
		uint16_t *p_validindex);

/**
 * \brief get the SID index in the HKConfig structure
 * \param struct_id housekeeping structure identifier
 * \param p_validindex pointer to the a valid index
 * \return 0 only if SID is valid, 1 if not
 */
error_code_t pus_service4_get_free_index(uint16_t *p_validindex);


/**
 * \brief get the SID index in the HKConfig structure
 * \param struct_id housekeeping structure identifier
 * \param p_validindex pointer to the a valid index
 * \return 0 only if SID is valid, 1 if not
 */

error_code_t pus_service4_reset_stats_index(uint16_t valid_index);

/**
 * \brief update the Parameter stats
 * \param param_id parameter identifier
 * \param pid_index index of the pid in the structure
 * \return 0 only if param_id is valid and the index contains the pid
 * stats, 1 if not
 */
error_code_t pus_service4_update_PID_stats(uint16_t valid_index);


#ifdef __cplusplus
}
#endif


#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE04_INCLUDE_PUS_SERVICE4_AUX_PUS_SERVICE4_UTILS_H_ */
