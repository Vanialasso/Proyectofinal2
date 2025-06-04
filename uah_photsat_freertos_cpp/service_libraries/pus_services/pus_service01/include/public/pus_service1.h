/*
 * icuasw_pus_service1.h
 *
 *  Created on: Dec 21, 2023
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

#ifndef PUBLIC__ICUASW_PUS_SERVICE1_H
#define PUBLIC__ICUASW_PUS_SERVICE1_H

#include <public/pus_tm_handler.h>
#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_accept_report.h"
#include "public/pus_tc_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief telecommand acceptation
 * \param ptc_handler pointer to the tc handler
 * \return acceptation report
 */

tc_accept_report_t pus_service1_tc_acceptation(
		tc_handler_t *ptc_handler);

/**
 * \brief TX a TM[1,1] telemetry
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_1(tc_handler_t *ptc_handler);

#define TM_1_2_CRC_ERROR_CODE 					2
#define TM_1_2_APID_NOT_VALID_ERROR_CODE 		3
#define TM_1_2_SOURCE_ID_NOT_VALID_ERROR_CODE 	4
#define TM_1_2_TYPE_NOT_VALID_ERROR_CODE 		5
#define TM_1_2_SUBTYPE_NOT_VALID_ERROR_CODE 	6

/**
 * \brief TX a TM[1,2] telemetry
 * \param ptc_handler pointer to the tc handler
 * \param tc_accept_report acceptation report
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_2(tc_handler_t *ptc_handler
		,tc_accept_report_t tc_accept_report);

/**
 * \brief TX a TM[1,3] telemetry
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_3(tc_handler_t *ptc_handler);

//Error with the TC APPDATA OR APPDATA FIELDS
#define TM_1_4_TC_X_Y_TC_SHORT_PACK_LENGTH		1
#define TM_1_4_TC_X_Y_TC_NOT_VALID_NUM_OF_INSTR	2
#define TM_1_4_TC_2_X_DEVICE_NOT_VALID  		3
#define TM_1_4_TC_2_X_DEVICE_ADDRESS_NOT_VALID  4
#define TM_1_4_TC_3_X_INVALID_SID 				5
#define TM_1_4_TC_5_X_INVALID_EV_ID				6
#define TM_1_4_TC_6_X_INVALID_MEMORY_ADDRESS	7
#define TM_1_4_TC_6_X_MEMORY_ID_READ_ONLY		8
#define TM_1_4_TC_6_X_OUT_OF_MEMORY_ID_LIMITS	9
#define TM_1_4_TC_11_X_SCHED_TC_REJECTED 		10
#define TM_1_4_TC_11_X_INVALID_SCHED_TIME		11
#define TM_1_4_TC_12_X_PMON_UNDEFINED			12
#define TM_1_4_TC_12_X_PMON_DEFINED				13
#define TM_1_4_TC_12_X_PMON_ENABLED				14
#define TM_1_4_TC_12_X_INVALID_PMONID			15
#define TM_1_4_TC_12_X_INVALID_PMON_DEFINITION	16
#define TM_1_4_TC_15_X_INVALID_PACKET_STORE		17
#define TM_1_4_TC_19_X_EV_ACTION_IS_ENABLED  	18
#define TM_1_4_TC_19_1_EV_ACTION_REJECTED 		19
#define TM_1_4_TC_19_X_EV_ACTION_NOT_DEFINED  	20
#define TM_1_4_TC_20_X_INVALID_PID 				21
#define TM_1_4_TC_20_X_PID_READ_ONLY_VIA_TC_20 	22
#define TM_1_4_TC_129_1_INVALID_OBSERV 			23
#define TM_1_4_TC_129_2_INVALID_KP_KI_KD		24

#define TM_1_8_TM_X_Y_TM_EXCEED_LIMIT_APPDATA	1
#define TM_1_8_TM_2_1_DEV_COMMAND_EXEC_ERROR	2
#define TM_1_8_TC_4_1_NOT_FREE_PID_STATS_CONFIG	3
#define TM_1_8_TC_4_7_PID_STATS_UNDEFINED		4
#define TM_1_8_TC_6_X_MEM_ACCESS_FAIL			5
#define TM_1_8_TC_11_X_SCHED_TC_POOL_EMPTY		6
#define TM_1_8_TC_19_1_MAX_EV_ACTIONS_REACHED 	7

/**
 * \brief TX a TM[1,4] short packet length
 * \param ptc_handler pointer to the tc handler
 * \param packet length
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_short_pack_length(
		tc_handler_t *ptc_handler);

/**
 * \brief TX a TM[1,4] number of instruction not valid
 * \param ptc_handler pointer to the tc handler
 * \param N TC Number of instructions
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_num_of_instr_not_valid(
		tc_handler_t *ptc_handler
		, uint8_t N);


/**
 * \brief TX a TM[1,4] DEVICE ADDRESS NOT VALID
 * \param ptc_handler pointer to the tc handler
 * \param device_address address not valid
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_device_address_not_valid(tc_handler_t *ptc_handler
			, uint32_t device_address);

/**
 * \brief TX a TM[1,4] INVALID SID
 * \param ptc_handler pointer to the tc handler
 * \param SID Structure ID
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_SID_not_valid(tc_handler_t *ptc_handler
			, uint16_t SID);


/**
 * \brief TX a TM[1,4] Mem Address not valid
 * \param ptc_handler pointer to the tc handler
 * \param MemID Memory ID
 * \param address address
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_mem_address_not_valid(tc_handler_t *ptc_handler
			, uint8_t MemID, uint32_t address);

/**
 * \brief TX a TM[1,4] Mem Address is read only
 * \param ptc_handler pointer to the tc handler
 * \param MemID Memory ID
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_mem_id_read_only(tc_handler_t *ptc_handler
			, uint8_t MemID);




/**
 * \brief TX a TM[1,4] INVALID EvID
 * \param ptc_handler pointer to the tc handler
 * \param EvID Event ID
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_EvID_not_valid(tc_handler_t *ptc_handler
		, uint16_t EvID);

/**
 * \brief TX a TM[1,4] INVALID PID
 * \param ptc_handler pointer to the tc handler
 * \param PID Parameter Identifier
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_PID_not_valid(tc_handler_t *ptc_handler
		, uint16_t PID);

/**
 * \brief TX a TM[1,4] PMON UNDEFINED
 * \param ptc_handler pointer to the tc handler
 * \param PMONID Parameter Monitoring Identifier
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_PMON_undefined(tc_handler_t *ptc_handler
		, uint16_t PMONID);

/**
 * \brief TX a TM[1,4] PMON DEFINED (it
 * \param ptc_handler pointer to the tc handler
 * \param PMONID Parameter Monitoring Identifier
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_PMON_undefined(tc_handler_t *ptc_handler
		, uint16_t PMONID);
/**
* \brief TX a TM[1,4] PMON ENABLED
* \param ptc_handler pointer to the tc handler
* \param PMONID Parameter Monitoring Identifier
* \return 0 if no error
*/

error_code_t pus_service1_tx_TM_1_4_PMON_enabled(tc_handler_t *ptc_handler
		, uint16_t PMONID);

/**
* \brief TX a TM[1,4] PMONID INVALID
* \param ptc_handler pointer to the tc handler
* \param PMONID Parameter Monitoring Identifier
* \return 0 if no error
*/

error_code_t pus_service1_tx_TM_1_4_PMONID_invalid(tc_handler_t *ptc_handler
		, uint16_t PMONID);

/**
* \brief TX a TM[1,4] PMON DEFINITION IS INVALID
* \param ptc_handler pointer to the tc handler
* \param PMONID Parameter Monitoring Identifier
* \return 0 if no error
*/

error_code_t pus_service1_tx_TM_1_4_PMON_definition_invalid(tc_handler_t *ptc_handler
		, uint16_t PMONID);

/**
 * \brief TX a TM[1,4] EvAction is enabled
 * \param ptc_handler pointer to the tc handler
 * \param EvID Event ID
 * \return 0 if no error
 */

error_code_t pus_service1_tx_TM_1_4_EvAction_enabled(tc_handler_t *ptc_handler,
		uint16_t EvID);



/**
 * \brief TX a TM[1,4] The action in the event action is not accepted
 * \param ptc_handler pointer to the tc handler
 * \param EvID Event ID whose action is rejected
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_EvAction_rejected(tc_handler_t *ptc_handler,
		uint16_t EvID);

/**
 * \brief TX a TM[1,4] The event action is not defined
 * \param ptc_handler pointer to the tc handler
 * \param EvID Event ID whose action has been refered
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_EvAction_undefined(tc_handler_t *ptc_handler,
		uint16_t EvID);



/**
 * \brief TX a TM[1,4] The PID is read only via TC
 * \param ptc_handler pointer to the tc handler
 * \param PID Parameter indentifier
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_PID_read_only_via_TC_20_X(tc_handler_t *ptc_handler,
		uint16_t PID);


/**
 * \brief TX a TM[1,4] The 129_1 observation is not valid
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_Invalid_129_1_Observation(tc_handler_t *ptc_handler);

/**
 * \brief TX a TM[1,4] The 129_2 PID is not valid
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_4_Invalid_129_2_Params(tc_handler_t *ptc_handler);



/**
 * \brief TX a TM[1,8] The stats is not defined
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_8_exceed_limit_appdata(tc_handler_t *ptc_handler);


/**
 * \brief TX a TM[1,8] to notify  device on_off command execution error
 * \param ptc_handler pointer to the tc handler
 * \param on_off_command command that causes the error
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_8_dev_comman_exec_error(tc_handler_t *ptc_handler
		,uint32_t on_off_command);

/**
 * \brief TX a TM[1,8] Not free stats config
 * \param ptc_handler pointer to the tc handler
 * \param PID PID whose stats cannot be defined
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_8_not_free_stats_config(tc_handler_t *ptc_handler,
		uint16_t PID);


/**
 * \brief TX a TM[1,8] The stats is not defined
 * \param ptc_handler pointer to the tc handler
 * \param PID whose stats does not exists
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_8_PID_stats_undefined(tc_handler_t *ptc_handler,
		uint16_t PID);

/**
 * \brief TX a TM[1,8] Mem Access Error
 * \param ptc_handler pointer to the tc handler
 * \param MemID Memory ID
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_8_mem_access_error(tc_handler_t *ptc_handler,
		uint8_t MemID);

/**
 * \brief TX a TM[1,8] Max number of EvActions are defined
 * \param ptc_handler pointer to the tc handler
 * \param EvID Event ID whose action cannot be added
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_8_Max_EvActions_defined(tc_handler_t *ptc_handler,
		uint16_t EvID);

/**
 * \brief TX a TM[1,7] telemetry
 * \param ptc_handler pointer to the tc handler
 * \return 0 if no error
 */
error_code_t pus_service1_tx_TM_1_7(tc_handler_t *ptc_handler);


#ifdef __cplusplus
}
#endif



#endif

