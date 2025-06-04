/*
 * pus_service_1_4_tc_x_x.c
 *
 *  Created on: Oct 25, 2024
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

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/tmtc_dyn_mem.h"
#include "public/pus_tc_handler.h"
#include "public/pus_tm_handler.h"

#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service01/include/pus_service1/aux_pus_service1_x_utils.h"

error_code_t pus_service1_tx_TM_1_4_short_pack_length(tc_handler_t *ptc_handler) {

	return pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
	TM_1_4_TC_X_Y_TC_SHORT_PACK_LENGTH, TCVerifStageExecStart,
			ptc_handler->tc_packet_header.packet_length);

}

error_code_t pus_service1_tx_TM_1_4_num_of_instr_not_valid(
		tc_handler_t *ptc_handler, uint8_t N) {

	return pus_service1_tx_TM_1_X_uint8_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_X_Y_TC_NOT_VALID_NUM_OF_INSTR, N);

}

error_code_t pus_service1_tx_TM_1_4_device_address_not_valid(
		tc_handler_t *ptc_handler, uint32_t device_address) {

	return pus_service1_tx_TM_1_X_uint32_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_2_X_DEVICE_ADDRESS_NOT_VALID,
			device_address);

}
error_code_t pus_service1_tx_TM_1_4_SID_not_valid(tc_handler_t *ptc_handler,
		uint16_t SID) {

	return pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_3_X_INVALID_SID, SID);

}

//#define TM_1_4_TC_6_X_INVALID_MEMORY_ADDRESS	8
//#define TM_1_4_TC_6_X_MEMORY_ID_READ_ONLY		9

error_code_t pus_service1_tx_TM_1_4_mem_address_not_valid(
		tc_handler_t *ptc_handler, uint8_t MemID, uint32_t address) {

	return (pus_service1_tx_TM_1_X_uint8_uint32_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_6_X_INVALID_MEMORY_ADDRESS, MemID,
			address));

}

error_code_t pus_service1_tx_TM_1_4_mem_id_read_only(tc_handler_t *ptc_handler,
		uint8_t MemID) {

	return (pus_service1_tx_TM_1_X_uint8_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_6_X_MEMORY_ID_READ_ONLY, MemID));

}
error_code_t pus_service1_tx_TM_1_4_EvID_not_valid(tc_handler_t *ptc_handler,
		uint16_t EvID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_5_X_INVALID_EV_ID, EvID));

}

error_code_t pus_service1_tx_TM_1_4_PID_not_valid(tc_handler_t *ptc_handler,
		uint16_t PID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_20_X_INVALID_PID, PID));

}

error_code_t pus_service1_tx_TM_1_4_PMON_undefined(tc_handler_t *ptc_handler,
		uint16_t PMONID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_12_X_PMON_UNDEFINED, PMONID));

}

error_code_t pus_service1_tx_TM_1_4_PMON_defined(tc_handler_t *ptc_handler,
		uint16_t PMONID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_12_X_PMON_DEFINED, PMONID));

}

error_code_t pus_service1_tx_TM_1_4_PMON_enabled(tc_handler_t *ptc_handler,
		uint16_t PMONID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_12_X_PMON_ENABLED, PMONID));

}

error_code_t pus_service1_tx_TM_1_4_PMONID_invalid(tc_handler_t *ptc_handler,
		uint16_t PMONID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_12_X_INVALID_PMONID, PMONID));

}

error_code_t pus_service1_tx_TM_1_4_PMON_definition_invalid(
		tc_handler_t *ptc_handler, uint16_t PMONID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_12_X_INVALID_PMON_DEFINITION,
			PMONID));

}

error_code_t pus_service1_tx_TM_1_4_EvAction_enabled(tc_handler_t *ptc_handler,
		uint16_t EvID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_19_X_EV_ACTION_IS_ENABLED, EvID));

}

error_code_t pus_service1_tx_TM_1_4_EvAction_rejected(tc_handler_t *ptc_handler,
		uint16_t EvID) {
	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_19_1_EV_ACTION_REJECTED, EvID));

}

error_code_t pus_service1_tx_TM_1_4_EvAction_undefined(
		tc_handler_t *ptc_handler, uint16_t EvID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_19_X_EV_ACTION_NOT_DEFINED, EvID));

}


error_code_t pus_service1_tx_TM_1_4_PID_read_only_via_TC_20_X(
		tc_handler_t *ptc_handler, uint16_t PID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_20_X_PID_READ_ONLY_VIA_TC_20, PID));

}


error_code_t pus_service1_tx_TM_1_4_Invalid_129_1_Observation(tc_handler_t *ptc_handler){

	pus_service1_tx_TM_1_X_no_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_129_1_INVALID_OBSERV);


}


error_code_t pus_service1_tx_TM_1_4_Invalid_129_2_Kp_Ki_Kd(tc_handler_t *ptc_handler){

	pus_service1_tx_TM_1_X_no_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_4_TC_129_2_INVALID_KP_KI_KD);


}




error_code_t pus_service1_tx_TM_1_8_Max_EvActions_defined(
		tc_handler_t *ptc_handler, uint16_t EvID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecCompletion, TM_1_8_TC_19_1_MAX_EV_ACTIONS_REACHED,
			EvID));
}

error_code_t pus_service1_tx_TM_1_8_exceed_limit_appdata(
		tc_handler_t *ptc_handler) {

	return (pus_service1_tx_TM_1_X_no_failure_data(ptc_handler,
			TCVerifStageExecCompletion, TM_1_8_TM_X_Y_TM_EXCEED_LIMIT_APPDATA));

}

error_code_t pus_service1_tx_TM_1_8_dev_comman_exec_error(
		tc_handler_t *ptc_handler, uint32_t on_off_command) {

	return (pus_service1_tx_TM_1_X_uint32_failure_data(ptc_handler,
			TCVerifStageExecCompletion, TM_1_8_TM_2_1_DEV_COMMAND_EXEC_ERROR,
			on_off_command));

}

error_code_t pus_service1_tx_TM_1_8_not_free_stats_config(
		tc_handler_t *ptc_handler, uint16_t PID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecCompletion,
			TM_1_8_TC_4_1_NOT_FREE_PID_STATS_CONFIG, PID));
}

error_code_t pus_service1_tx_TM_1_8_PID_stats_undefined(
		tc_handler_t *ptc_handler, uint16_t PID) {

	return (pus_service1_tx_TM_1_X_uint16_failure_data(ptc_handler,
			TCVerifStageExecStart, TM_1_8_TC_4_7_PID_STATS_UNDEFINED, PID));
}


error_code_t pus_service1_tx_TM_1_8_mem_access_error(tc_handler_t *ptc_handler,
		uint8_t MemID){

	return (pus_service1_tx_TM_1_X_uint8_failure_data(ptc_handler,
				TCVerifStageExecCompletion,
				TM_1_8_TC_6_X_MEM_ACCESS_FAIL, MemID));

}
