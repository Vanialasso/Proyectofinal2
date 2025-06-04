/*
 * aux_pus_service19_x_utils.h
 *
 *  Created on: Nov 12, 2024
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

#ifndef SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE19_INCLUDE_PUS_SERVICE19_AUX_PUS_SERVICE19_UTILS_H_
#define SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE19_INCLUDE_PUS_SERVICE19_AUX_PUS_SERVICE19_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "public/basic_types.h"
#include "public/pus_tc_handler.h"
#include "public/pus_service19.h"

typedef uint16_t ev_action_id_t;

/* \brief calculate the number of actions of 5_1 event pending
 * \return return the number of 5_1 event actions pending to be executed
*/
uint8_t pus_service19_pending_ev_5_1_actions();

/* \brief calculate the number of actions of 5_1 event pending
 * \return return the number of 5_2 event actions pending to be executed
*/
uint8_t pus_service19_pending_ev_5_2_actions();

/* \brief calculate the number of actions of 5_1 event pending
 * \return return the number of 5_3 event actions pending to be executed
*/
uint8_t pus_service19_pending_ev_5_3_actions();

/* \brief calculate the number of actions of 5_1 event pending
 * \return return the number of 5_4 event actions pending to be executed
*/
uint8_t pus_service19_pending_ev_5_4_actions();


/* \brief is ev_action defined for this EvID?
 * \param EvID event identifier
 * \param p_ev_action_id pointer where the event action identifier is copied
 * \param p_enabled pointer where the event action status is copied
 * \return true if ev action is defined
*/

bool_t pus_service19_is_ev_action_defined(uint16_t EvID,
		ev_action_id_t *p_ev_action_id, bool_t *p_enabled);

/* \brief enable ev_action defined for this EvID
 * \param EvID event identifier
 * \param ev_action_id the event action identifier
 * \return 0 if no error
*/
error_code_t pus_service19_disable_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id);

/* \brief disable ev_action defined for this EvID
 * \param EvID event identifier
 * \param ev_action_id the event action identifier
 * \return 0 if no error
*/
error_code_t pus_service19_enable_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id);

/* \brief delete ev_action defined for this EvID
 * \param EvID event identifier
 * \param ev_action_id the event action identifier
 * \return 0 if no error
*/
error_code_t pus_service19_delete_ev_action(uint16_t EvID,
		ev_action_id_t ev_action_id);

/* \brief get the next free action id
 * \param p_ev_action_id pointer where the event action identifier is copied
 * \return error. 0 if free ev action has been found so p_ev_action_id is valid
*/
error_code_t pus_service19_get_free_ev_action_id(
		ev_action_id_t *p_ev_action_id);

/* \brief get ev action which identifier is ev_action_id
 * \param ev_action_id the event action identifier
 * \param p_action_bytes pointer to memory where the action is copied
 * \param p_num_of_bytes pointer to uint16_t where the value of number of bytes
 * is copied
 * \return error. 0 means ev_action_id and pointers are valid
*/

error_code_t pus_service19_get_ev_action(ev_action_id_t ev_action_id,
		uint8_t *p_action_bytes, uint16_t *p_num_of_bytes);

/* \brief set ev action which identifier is ev_action_id
 * \param EvID event identifier
 * \param ev_action_id the event action identifier
 * \param p_action_bytes pointer to memory from the action is copied
 * \param p_num_of_bytes pointer to uint16_t with the value of number of bytes
 * \return error. 0 means ev_action_id and pointers are valid
*/
error_code_t pus_service19_set_ev_action(uint16_t evID,
		ev_action_id_t ev_action_id,
		uint8_t *p_action_bytes, uint16_t num_of_bytes);

/* \brief add ev action which identifier is ev_action_id to queue
 * \param ev_action_id the event action identifier
 * \return error. 0 means ev_action_id and pointers are valid
*/
error_code_t pus_service19_add_ev_action_to_queue(uint16_t evID,
		ev_action_id_t ev_action_id);

/* \brief extract next pending event 5_1 action
 * \return error if action cannot be extracted
*/
error_code_t pus_service19_extract_ev_5_1_action(tc_handler_t *ptc_handler);

/* \brief extract next pending event 5_2 action
 * \return error if action cannot be extracted
*/
error_code_t pus_service19_extract_ev_5_2_action(tc_handler_t *ptc_handler);

/* \brief extract next pending event 5_3 action
 * \return error if action cannot be extracted
*/
error_code_t pus_service19_extract_ev_5_3_action(tc_handler_t *ptc_handler);

/* \brief extract next pending event 5_4 action
 * \return error if action cannot be extracted
*/
error_code_t pus_service19_extract_ev_5_4_action(tc_handler_t *ptc_handler);



#ifdef __cplusplus
}
#endif

#endif /* SERVICE_LIBRARIES_PUS_SERVICES_PUS_SERVICE19_INCLUDE_PUS_SERVICE19_AUX_PUS_SERVICE19_UTILS_H_ */
