/*
 * aux_pus_service1_utils.c
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

#include "../../pus_service01/include/pus_service1/aux_pus_service1_x_utils.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/tmtc_dyn_mem.h"

#include "../../pus_service01/include/public/pus_service1.h"

error_code_t pus_service1_build_TM_1_2_failure_notice(tc_handler_t *ptc_handler,
		tm_handler_t *pvalid_tm_handler, tc_accept_report_t tc_accept_report) {

	error_code_t error = 0;

	if (ptc_handler) {

		switch (tc_accept_report.accept_status) {

		case (TCAcceptationCRCError):

			//TM_1_2_CRC_ERROR_CODE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
			TM_1_2_CRC_ERROR_CODE);

			//TC PACK_ERROR_CTRL -> TM
			tm_handler_append_uint16_appdata_field(pvalid_tm_handler,
					ptc_handler->tc_packet_err_ctrl);

			//CALCULATED PACK_ERROR_CTRL -> TM
			tm_handler_append_uint16_appdata_field(pvalid_tm_handler,
					tc_accept_report.cal_packet_err_ctrl);

			break;

		case (TCAcceptationAPIDError):

			//TM_1_2_APID_NOT_VALID_ERROR_CODE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
			TM_1_2_APID_NOT_VALID_ERROR_CODE);

			break;

		case (TCAcceptationSourceIDError):

			//TM_1_2_SOURCE_ID_NOT_VALID_ERROR_CODE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
			TM_1_2_SOURCE_ID_NOT_VALID_ERROR_CODE);

			//SOURCE ID -> TM
			tm_handler_append_uint16_appdata_field(pvalid_tm_handler,
					ptc_handler->tc_df_header.sourceID);

			break;

		case (TCAcceptationTypeError):

			//TM_1_2_TYPE_NOT_VALID_ERROR_CODE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
			TM_1_2_TYPE_NOT_VALID_ERROR_CODE);

			//TYPE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
					ptc_handler->tc_df_header.type);

			break;

		case (TCAcceptationSubTypeError):

			//TM_1_2_SUBTYPE_NOT_VALID_ERROR_CODE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
			TM_1_2_SUBTYPE_NOT_VALID_ERROR_CODE);

			//SUBTYPE -> TM
			tm_handler_append_uint8_appdata_field(pvalid_tm_handler,
					ptc_handler->tc_df_header.subtype);

			break;

		default:

			error = 2;
			break;

		}
	} else {

		error = 1;
	}

	return error;
}

error_code_t pus_service1_build_TM_1_X_reqID(tc_handler_t *ptc_handler,
		tm_handler_t *p_tm_handler) {

	error_code_t error = 0;

	if (ptc_handler) {

		//PACK_ID -> TM
		error = tm_handler_append_uint16_appdata_field(p_tm_handler,
				ptc_handler->tc_packet_header.packet_id);

		//PACK_SEQ CTRL -> TM
		error += tm_handler_append_uint16_appdata_field(p_tm_handler,
				ptc_handler->tc_packet_header.packet_seq_ctrl);

	} else {
		error = 1;
	}
	return error;

}

error_code_t pus_service1_tx_TM_1_X_reqID(tc_handler_t *ptc_handler,
		uint8_t type, uint8_t subtype) {

	error_code_t error = 0;

	tm_handler_t tm_handler;

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//ReqID -> TM
		error = pus_service1_build_TM_1_X_reqID(ptc_handler, &tm_handler);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, type, subtype);
		}
		//free memory
		tm_handler_free_mem(&tm_handler);
	}

	return error;
}

uint8_t pus_service1_failure_tc_subtype(tc_verif_stage_t verif_stage){

	uint8_t tc_subtype;

	switch (verif_stage){

	case(TCVerifStageAcceptation):
		tc_subtype=2;
		break;
	case(TCVerifStageExecStart):
		tc_subtype=4;
		break;

	case(TCVerifStageExecProgress):
		tc_subtype=6;
		break;

	case(TCVerifStageExecCompletion):
		tc_subtype=8;
		break;

	}

	return tc_subtype;

}

error_code_t pus_service1_tx_TM_1_X_no_failure_data(tc_handler_t *ptc_handler
		,tc_verif_stage_t verif_stage,uint8_t failure_code){

	error_code_t error = 0;

	tm_handler_t tm_handler;

	uint8_t tc_subtype=pus_service1_failure_tc_subtype(verif_stage);

	//Do no check start execution ack_enabled

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//Request ID ->TM
		error = pus_service1_build_TM_1_X_reqID(ptc_handler, &tm_handler);

		//Failure Code ID ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
				failure_code);
		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 1, tc_subtype);
		}

		//free memory
		tm_handler_free_mem(&tm_handler);
	}
	return error;

}
error_code_t pus_service1_tx_TM_1_X_uint8_failure_data(tc_handler_t *ptc_handler
		,tc_verif_stage_t verif_stage,uint8_t failure_code, uint8_t failure_data){

	error_code_t error = 0;

	tm_handler_t tm_handler;

	uint8_t tc_subtype=pus_service1_failure_tc_subtype(verif_stage);

	//Do no check start execution ack_enabled

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//Request ID ->TM
		error = pus_service1_build_TM_1_X_reqID(ptc_handler, &tm_handler);

		//Failure Code ID ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
				failure_code);

		//UINT8 Failure Data ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
				failure_data);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 1, tc_subtype);
		}

		//free memory
		tm_handler_free_mem(&tm_handler);
	}
	return error;

}


error_code_t pus_service1_tx_TM_1_X_uint16_failure_data(tc_handler_t *ptc_handler
		,tc_verif_stage_t verif_stage,uint8_t failure_code, uint16_t failure_data){

	error_code_t error = 0;

	tm_handler_t tm_handler;

	uint8_t tc_subtype=pus_service1_failure_tc_subtype(verif_stage);

	//Do no check start execution ack_enabled

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//Request ID ->TM
		error = pus_service1_build_TM_1_X_reqID(ptc_handler, &tm_handler);

		//Failure Code ID ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
				failure_code);

		//UINT16 Failure Data ->TM
		error += tm_handler_append_uint16_appdata_field(&tm_handler,
				failure_data);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 1, tc_subtype);
		}

		//free memory
		tm_handler_free_mem(&tm_handler);
	}
	return error;

}

error_code_t pus_service1_tx_TM_1_X_uint32_failure_data(tc_handler_t *ptc_handler
		,tc_verif_stage_t verif_stage,uint8_t failure_code, uint32_t failure_data){

	error_code_t error = 0;

	tm_handler_t tm_handler;

	uint8_t tc_subtype=pus_service1_failure_tc_subtype(verif_stage);

	//Do no check start execution ack_enabled

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//Request ID ->TM
		error = pus_service1_build_TM_1_X_reqID(ptc_handler, &tm_handler);

		//Failure Code ID ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
				failure_code);

		//UINT32 Failure Data ->TM
		error += tm_handler_append_uint32_appdata_field(&tm_handler,
				failure_data);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 1, tc_subtype);
		}

		//free memory
		tm_handler_free_mem(&tm_handler);
	}
	return error;

}

error_code_t pus_service1_tx_TM_1_X_uint8_uint32_failure_data(
		tc_handler_t *ptc_handler, tc_verif_stage_t verif_stage,
		uint8_t failure_code, uint8_t failure_data1, uint32_t failure_data2){
	error_code_t error = 0;

	tm_handler_t tm_handler;

	uint8_t tc_subtype=pus_service1_failure_tc_subtype(verif_stage);

	//Do no check start execution ack_enabled

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//Request ID ->TM
		error = pus_service1_build_TM_1_X_reqID(ptc_handler, &tm_handler);

		//Failure Code ID ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
				failure_code);

		//UINT8 Failure Data1 ->TM
		error += tm_handler_append_uint8_appdata_field(&tm_handler,
					failure_data1);

		//UINT32 Failure Data2 ->TM
		error += tm_handler_append_uint32_appdata_field(&tm_handler,
				failure_data2);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 1, tc_subtype);
		}

		//free memory
		tm_handler_free_mem(&tm_handler);
	}
	return error;



}

