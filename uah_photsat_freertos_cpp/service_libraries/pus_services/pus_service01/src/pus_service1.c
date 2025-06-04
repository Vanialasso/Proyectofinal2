/*
 * icuasw_pus_service1.cpp
 *
 *  Created on: Dec 21, 2016
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

#include "../../pus_service01/include/public/pus_service1.h"

#include "public/ccsds_pus.h"
#include "public/crc.h"
#include "public/tmtc_dyn_mem.h"
#include "pus_service1/aux_pus_service1_x_utils.h"

#include "../../pus_service01/include/pus_service1/aux_pus_service1_x_utils.h"

tc_accept_report_t pus_service1_tc_acceptation(tc_handler_t *ptc_handler) {

	tc_accept_report_t accept_report;

	uint16_t crc_offset;


	if (ptc_handler) {
		//Get Headers and pack error control from tc bytes

		crc_offset = ptc_handler->tc_packet_header.packet_length
				+ UAH_PUS_TM_TC_PACKET_HEADER_SIZE - 1;

		//Cal CRC
		accept_report.cal_packet_err_ctrl = cal_crc_16(
				ptc_handler->raw_tc_mem_descriptor.p_tc_bytes, crc_offset);

		//Check errors
		if (accept_report.cal_packet_err_ctrl
				!= ptc_handler->tc_packet_err_ctrl) {

			accept_report.accept_status = TCAcceptationCRCError;

		} else if ( UAH_APID
				!= ccsds_pus_tc_get_APID(
						ptc_handler->tc_packet_header.packet_id)) {

			accept_report.accept_status = TCAcceptationAPIDError;

		} else if ( UAH_SOURCE_ID != ptc_handler->tc_df_header.sourceID) {

			accept_report.accept_status = TCAcceptationSourceIDError;

		} else if (pus_services_sat_in_observation_mode()){

			accept_report.accept_status = TCAcceptationInObservationModeError;

		}else {


			accept_report.accept_status = TCAcceptationOK;

			switch (ptc_handler->tc_df_header.type) {
			case (9):
				switch (ptc_handler->tc_df_header.subtype) {
				case (129):
						break;
				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (17):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
					break;
				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (2):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
					break;

				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (3):
				switch (ptc_handler->tc_df_header.subtype) {
				case (5):
				case (6):
				case (31):
					break;

				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (4):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
				case (6):
				case (7):
					break;
				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;

			case (5):
				switch (ptc_handler->tc_df_header.subtype) {
				case (5):
				case (6):
					break;
				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (12):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
				case (2):
				case (5):
				case (6):
					break;
				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (19):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
				case (2):
				case (4):
				case (5):
					break;
				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (20):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
				case (3):
					break;

				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			case (128):
				switch (ptc_handler->tc_df_header.subtype) {
				case (1):
					break;

				default:
					accept_report.accept_status = TCAcceptationSubTypeError;
				}
				break;
			//TODO Accept ST[129] TCs

			default:
				//TC is not accepted
				accept_report.accept_status = TCAcceptationTypeError;
				break;

			}
		}
	}

	return accept_report;

}

error_code_t pus_service1_tx_TM_1_1(tc_handler_t *ptc_handler) {

	error_code_t error = 0;

	if (tc_handler_is_tc_accept_ack_enabled(ptc_handler)) {

		error = pus_service1_tx_TM_1_X_reqID(ptc_handler, 1, 1);
	}

	return error;
}

error_code_t pus_service1_tx_TM_1_2(tc_handler_t *pvalid_tc_handler,
		tc_accept_report_t tc_accept_report) {

	error_code_t error = 0;

	tm_handler_t tm_handler;

	//Do no check accept_ack_enabled

	//Alloc memory
	error = tm_handler_mem_alloc_and_startup(&tm_handler);

	if (0 == error) {

		//Request ID ->TM
		error = pus_service1_build_TM_1_X_reqID(pvalid_tc_handler, &tm_handler);

		//Error Code ID ->TM
		error += pus_service1_build_TM_1_2_failure_notice(pvalid_tc_handler,
				&tm_handler, tc_accept_report);

		//close and Tx only in no error
		if (0 == error) {

			error = tm_handler_close_and_tx(&tm_handler, 1, 2);
		}

		//free memory
		tm_handler_free_mem(&tm_handler);

	}
	return error;

}

error_code_t pus_service1_tx_TM_1_3(tc_handler_t *ptc_handler) {

	error_code_t error = 0;

	//If start exec flag is enable
	if (tc_handler_is_tc_start_exec_ack_enabled(ptc_handler)) {

		error = pus_service1_tx_TM_1_X_reqID(ptc_handler, 1, 3);
	}

	return error;

}

error_code_t pus_service1_tx_TM_1_7(tc_handler_t *ptc_handler) {

	error_code_t error = 0;

	//If start exec flag is enable
	if (tc_handler_is_tc_completion_exec_ack_enabled(ptc_handler)) {

		error = pus_service1_tx_TM_1_X_reqID(ptc_handler, 1, 7);
	}

	return error;

}

