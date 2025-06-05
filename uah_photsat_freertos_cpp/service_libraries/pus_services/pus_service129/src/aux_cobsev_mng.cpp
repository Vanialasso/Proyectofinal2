/*
 * aux_cobsev_mng.cpp
 *
 *  Created on: May 7, 2025
 *      Author: opolo70
 */

#include "public/config.h"
#include "public/pus_tc_handler.h"
#include "public/pus_service1.h"
#include "public/pus_service129.h"

#include "pus_service129/aux_cobserv_mng.h"

#include "public/attitude_pid_ctrl.h"
#include "public/emu_sat_attitude.h"

//TODO Completar el código de los métodos de la clase CObservationMng
int CImageCounter;
int CNumImages;
int CImageInterval;
float CTargetPitch;
float CTargetYaw;

void ObsMng::EDROOM_CTX_Top_0::Finit() {
    CImageCounter = 0;
}
void ObsMng::EDROOM_CTX_Top_0::FExecObsMng_TC() {
    CTargetPitch     = Msg->data.pitch;
    CTargetYaw       = Msg->data.yaw;
    CNumImages       = Msg->data.num_images;
    CImageInterval   = Msg->data.interval;
}
    void ObsMng::EDROOM_CTX_Top_0::FToObservation() {
        CImageCounter = 0;
        pus_service129_is_observation_ready();
    }
    void ObsMng::EDROOM_CTX_Top_0::FProgTakeImage() {
        Pr_Time time;
        time.GetTime();                    // Obtener tiempo actual
        time.Add(CImageInterval, 0);      // Sumar intervalo (en segundos)
        ObservTimer.InformAt(time);       // Programar evento EvTakeImage
    }
    void ObsMng::EDROOM_CTX_Top_0::FProgAttitudeCtrl() {
        Pr_Time time;
        time.GetTime();
        time.Add(0, 100000); // 100 ms = 100,000 us
        ObservTimer.InformAt(time); // Programar evento EvDoAttitudeCtrl
    }
    void ObsMng::EDROOM_CTX_Top_0::FDoAttitudeCtrl() {
        pus_service129_do_attitude_ctrl(CTargetPitch, CTargetYaw);
    }
    void ObsMng::EDROOM_CTX_Top_0::FTakeImage() {
        pus_service129_take_image(CImageCounter);
        CImageCounter++;

        if (CImageCounter < CNumImages) {
            FProgTakeImage();
        } else {
            FEndObservation();
        }
    }
    void ObsMng::EDROOM_CTX_Top_0::FEndObservation() {
        VNextTimeOut.GetTime(); // Esto es opcional: actualiza el tiempo
        pus_service129_end_observation();
    }
