/*
 * icuasw_pus_service129.h
 *
 *  Created on: May 7, 2024
 *      Author: opolo70
 */

#ifndef SERVICE_LIBRARIES_ICUASW_PUS_SERVICES_INCLUDE_ICUASW_PUS_SERVICES_ICUASW_PUS_SERVICE129_H_
#define SERVICE_LIBRARIES_ICUASW_PUS_SERVICES_INCLUDE_ICUASW_PUS_SERVICES_ICUASW_PUS_SERVICE129_H_

#include "public/config.h"
#include "public/basic_types.h"

#include "public/pus_tc_handler.h"
#include "public/pus_tm_handler.h"
#include "public/pus_sys_data_pool.h"



#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief init service 129
 */

void pus_service129_startup();

/**
 * \brief do attitude ctrl
 */
void pus_service129_do_attitude_ctrl();


/**
 * \brief executes a TC[129,X] telecommand
 * \param ptc_handler pointer to the tc handler
 */
void pus_service129_exec_tc(tc_handler_t *ptc_handler);


/**
 * \brief Start Observation
 *
 */

void pus_service129_start_observation();



/**
 * \brief Take Image
 *
 */

void pus_service129_take_image();


/**
 * \brief Start Observation
 * \return true if it is last image
 *
 */
bool_t pus_service129_is_last_image();

/**
 * \brief End Observation
 *
 */

void pus_service129_end_observation();


/**
 * \brief Is Observation Ready
 *
 */
bool_t pus_service129_is_observation_ready();


#ifdef __cplusplus
}
#endif



/*


 static class UAHExpEmuKinematic mUAHExpEmulator;

protected:
  static void ExecTC( CDTCHandler &tcHandler, CDTMList &tmList);

  static void Exec129_1TC( CDTCHandler &tcHandler, CDTMList &tmList);
  static void Exec129_2TC( CDTCHandler &tcHandler, CDTMList &tmList);


  static float LimitActuator(float F, float Limit);
public:
  static void GuidanceControl();

};

*/

#endif /* SERVICE_LIBRARIES_ICUASW_PUS_SERVICES_INCLUDE_ICUASW_PUS_SERVICES_ICUASW_PUS_SERVICE129_H_ */
