/*
 * aux_pus_service19_exec_tc.c
 *
 *  Created on: Nov 13, 2024
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
#include <public/pus_tc_handler.h>
#include "public/config.h"
#include "pus_service19/aux_pus_service19_utils.h"
#include "pus_service19/aux_pus_service19_exec_tc.h"
#include "public/pus_service19.h"
#include "../../pus_service01/include/public/pus_service1.h"
#include "../../pus_service05/include/public/pus_service5.h"

void pus_service19_exec_TC_19_1(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t EvID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> EvID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &EvID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is not valid this EvID?
			if (!pus_service5_is_EvID_valid(EvID)) {
				//If not valid, EvID
				pus_service1_tx_TM_1_4_EvID_not_valid(ptc_handler, EvID);

			} else {

				ev_action_id_t ev_action_id;
				bool_t is_enabled;
				bool_t is_defined;

				is_defined = pus_service19_is_ev_action_defined(EvID,
						&ev_action_id, &is_enabled);

				if (is_defined) {

					if (is_enabled) {

						pus_service1_tx_TM_1_4_EvAction_enabled(ptc_handler,
								EvID);

						error = 1;
					}
				}

				if (!error) {

					tc_handler_t action_tc_handler;
					tc_mem_descriptor_t action_tc_mem_descriptor;
					tc_accept_report_t tc_accept_report;

					tc_handler_get_tc_mem_descriptor_from_remaining_appdata(
							ptc_handler, &action_tc_mem_descriptor);

					tc_handler_build_from_descriptor(&action_tc_handler,
							action_tc_mem_descriptor);

					tc_accept_report = pus_service1_tc_acceptation(
							&action_tc_handler);

					if (TCAcceptationOK == tc_accept_report.accept_status) {

						pus_service1_tx_TM_1_3(ptc_handler);

						if (!is_defined)
							error = pus_service19_get_free_ev_action_id(
									&ev_action_id);

						if (!error) {

							pus_service19_set_ev_action(EvID, ev_action_id,
									action_tc_handler.raw_tc_mem_descriptor.p_tc_bytes,
									action_tc_handler.raw_tc_mem_descriptor.tc_num_bytes);

							pus_service19_disable_ev_action(EvID, ev_action_id);

							pus_service1_tx_TM_1_7(ptc_handler);

						} else {

							pus_service1_tx_TM_1_8_Max_EvActions_defined(
									ptc_handler, EvID);

						}
					} else {

						pus_service1_tx_TM_1_4_EvAction_rejected(ptc_handler,
								EvID);
					}
				}

			}
		}
	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

void pus_service19_exec_TC_19_2(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t EvID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> EvID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &EvID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is not valid this EvID?
			if (!pus_service5_is_EvID_valid(EvID)) {
				//If not valid, EvID
				pus_service1_tx_TM_1_4_EvID_not_valid(ptc_handler, EvID);

			} else {

				ev_action_id_t ev_action_id;
				bool_t is_enabled;
				bool_t is_defined;

				is_defined = pus_service19_is_ev_action_defined(EvID,
						&ev_action_id, &is_enabled);

				if (is_defined) {

					if (is_enabled) {

						pus_service1_tx_TM_1_4_EvAction_enabled(ptc_handler,
								EvID);

						error = 1;

					} else {

						pus_service1_tx_TM_1_3(ptc_handler);

						pus_service19_delete_ev_action(EvID, ev_action_id);

						pus_service1_tx_TM_1_7(ptc_handler);

					}

				} else {

					pus_service1_tx_TM_1_4_EvAction_undefined(ptc_handler,
							EvID);
				}

			}
		}
	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

void pus_service19_exec_TC_19_4(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t EvID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> EvID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &EvID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is not valid this EvID?
			if (!pus_service5_is_EvID_valid(EvID)) {
				//If not valid, EvID
				pus_service1_tx_TM_1_4_EvID_not_valid(ptc_handler, EvID);

			} else {

				ev_action_id_t ev_action_id;
				bool_t is_enabled;
				bool_t is_defined;

				is_defined = pus_service19_is_ev_action_defined(EvID,
						&ev_action_id, &is_enabled);

				if (is_defined) {

					pus_service1_tx_TM_1_3(ptc_handler);

					pus_service19_enable_ev_action(EvID, ev_action_id);

					pus_service1_tx_TM_1_7(ptc_handler);

				} else {

					pus_service1_tx_TM_1_4_EvAction_undefined(ptc_handler,
							EvID);
				}

			}
		}
	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

void pus_service19_exec_TC_19_5(tc_handler_t *ptc_handler) {

	error_code_t error;

	uint8_t N;
	uint16_t EvID;

	// TC -> N
	error = tc_handler_get_uint8_appdata_field(ptc_handler, &N);

	// TC -> EvID
	error += tc_handler_get_uint16_appdata_field(ptc_handler, &EvID);

	if (error) {

		// error in pack length
		pus_service1_tx_TM_1_4_short_pack_length(ptc_handler);

	} else {

		if (1 != N) {

			error = pus_service1_tx_TM_1_4_num_of_instr_not_valid(ptc_handler,
					N);
		} else {

			//Is not valid this EvID?
			if (!pus_service5_is_EvID_valid(EvID)) {
				//If not valid, EvID
				pus_service1_tx_TM_1_4_EvID_not_valid(ptc_handler, EvID);

			} else {

				ev_action_id_t ev_action_id;
				bool_t is_enabled;
				bool_t is_defined;

				is_defined = pus_service19_is_ev_action_defined(EvID,
						&ev_action_id, &is_enabled);

				if (is_defined) {

					pus_service1_tx_TM_1_3(ptc_handler);

					pus_service19_disable_ev_action(EvID, ev_action_id);

					pus_service1_tx_TM_1_7(ptc_handler);

				} else {

					pus_service1_tx_TM_1_4_EvAction_undefined(ptc_handler,
							EvID);
				}

			}
		}
	}

	//free memory
	tc_handler_free_memory(ptc_handler);

}

