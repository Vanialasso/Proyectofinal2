/*
 * tc_rate_ctrl.cpp
 *
 *  Created on: Apr 27, 2024
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

#include "public/tc_rate_ctrl.h"
#include "public/edroombp.h"

#define MAX_NUM_OF_TC_PER_SEC 			10

//Elements that need a critical section
//The producer and consumer can handle
static uint16_t TCDispatchTimeHead = 0;
static uint16_t TCDispatchTimeNumOfElements = 0;

//These elements does not need a critical section
static Pr_Time TCDispatchTimeQueueElements[MAX_NUM_OF_TC_PER_SEC];

static bool    TCRateExceeded=false;


Pr_Mutex mutex;

void ExtractHead() {

	TCDispatchTimeHead = (TCDispatchTimeHead + 1) % MAX_NUM_OF_TC_PER_SEC;
	TCDispatchTimeNumOfElements--;
}

bool IsFull() {

	return (TCDispatchTimeNumOfElements == MAX_NUM_OF_TC_PER_SEC);
}




void AddElement(Pr_Time time, bool isAction) {

	uint16_t tailIndex;

	tailIndex = (TCDispatchTimeHead + TCDispatchTimeNumOfElements)
			% MAX_NUM_OF_TC_PER_SEC;

	TCDispatchTimeQueueElements[tailIndex] = time;

	TCDispatchTimeNumOfElements++;

}

void RxTC_RateCtrl() {

	Pr_Time time;
	uint16_t  MaxRxTC;
	uint16_t  TCDispached;

	//Maximum of Rx TC
	MaxRxTC=MAX_NUM_OF_TC_PER_SEC;

	mutex.Wait();

	TCDispached=TCDispatchTimeNumOfElements ;

	//Check Rate
	if (TCDispached >=MaxRxTC) {

		//Check Head Time Stamp
		Pr_Time validTime;

		validTime=TCDispatchTimeQueueElements[TCDispatchTimeHead];
		validTime+=Pr_Time(1,0); //Add a Second to the fist TC of the Queue
		time.GetTime(); //Current Time
		//Aux is a second after
		if(time < validTime){

			TCRateExceeded=true;
			mutex.Signal();

			//Containment
			Pr_DelayAt(validTime);

			mutex.Wait();

		}

		ExtractHead();

	}

	time.GetTime(); //Time when TC has been dispatched
	AddElement(time,false);
	mutex.Signal();

}


bool RxTC_TCRateExceeded() {

	bool tcRateExceeded;
	mutex.Wait();

	tcRateExceeded=TCRateExceeded;
	TCRateExceeded=false;

	mutex.Signal();

	return tcRateExceeded;
}



