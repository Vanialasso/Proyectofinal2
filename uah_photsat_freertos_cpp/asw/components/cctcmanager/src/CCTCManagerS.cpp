

#include <public/cctcmanager_iface_v1.h>



	// ******************* HANDLING IRQ 18********************

Pr_IRQEvent	CCTCManager::EDROOMEventIRQ18(18);
Pr_SemaphoreBin	CCTCManager::EDROOMSemEndIRQ18(0);


		// ******************* DATA ***************

CDTCMemDescriptor	CCTCManager::	EDROOMVarIRQ18;


		// ******************* DATA POOL *******

CCTCManager::CEDROOMPOOLIRQ18CDTCMemDescriptor	CCTCManager::EDROOMPoolIRQ18;


		// ******************* Aux IRQ Handler **************

void 	CCTCManager::EDROOMIRQ18HandlerTopHalfFunction(void){

	bool EDROOMIRQ18BottomHalfSignal=true;
	#ifdef NEXYS_PLATFORM 

	 

	EDROOMIRQ18BottomHalfSignal=EDROOMVarIRQ18.HandleIRQ();

	 

	#endif

	if (EDROOMIRQ18BottomHalfSignal)
		EDROOMEventIRQ18.Signal();

}



		// ******************* IRQ Handler **************

Pr_IRQHandler_RetType	CCTCManager::EDROOMIRQ18Handler(void){

	EDROOMIRQ18HandlerTopHalfFunction();

}



		// ******************* IRQ Idle Handler **************

Pr_IRQHandler_RetType	CCTCManager::EDROOMIRQ18IdleHandler(void){

}



		// ******************* Bottom Half Task **************

Pr_TaskRV_t 	CCTCManager::EDROOMIRQ18BottomHalfTask(Pr_TaskP_t){

	bool endTask=false;

	do
	{

		EDROOMEventIRQ18.Wait();

		bool EDROOMIRQ18SendMsgToCmp=true;

		if(!EDROOMSemEndIRQ18.WaitCond()){

			 

			RxTC_RateCtrl();

			 

			SC_Channel_GetNextTC(EDROOMVarIRQ18);

			 

			SC_Channel_HandlePendingTCs();

			if(EDROOMIRQ18SendMsgToCmp){
				CDTCMemDescriptor	*pEDROOMVarIRQ;

				pEDROOMVarIRQ=EDROOMPoolIRQ18.AllocData();

				*pEDROOMVarIRQ=EDROOMVarIRQ18;

				RxTC.NewIRQMsg(EDROOMIRQsignal, pEDROOMVarIRQ, &EDROOMPoolIRQ18);

			}
			Pr_IRQManager::EnableIRQ(18);

		}else endTask=1;

	}while(!endTask);

}



		// ******************* IRQPort **************

CEDROOMIRQInterface CCTCManager::RxTC( 
		CCTCManager::EDROOMIRQ18Handler 
		,CCTCManager::EDROOMIRQ18IdleHandler 
		,CCTCManager::EDROOMEventIRQ18
		,CCTCManager::EDROOMSemEndIRQ18
		,18 );




// ******************************* CONSTRUCTOR ************************************



CCTCManager::CCTCManager(TEDROOMComponentID id,
		TEDROOMUInt32 roomNumMaxMens,
		TEDROOMPriority roomtaskPrio,
		TEDROOMStackSizeType roomStack,
		CEDROOMMemory *pActorMemory ) : 

		CEDROOMComponent(id,EDROOMprioMINIMUM+1,roomNumMaxMens,
				roomtaskPrio,roomStack, pActorMemory ),

		// ***************   BOTTOM HALF IRQ TASKS  ********************

		EDROOMIRQ18BottomHalfT( EDROOMIRQ18BottomHalfTask,EDROOMprioURGENT,256),

		// ***************	Top State  *****************

		edroomTopState(*this,pActorMemory)


{


}


//**************************************************************************
//**************************   PUBLIC METHODS  *****************************



//************************** EDROOMConfig **********************************


int CCTCManager::EDROOMConfig()
{


 return 0; 
}



//************************** EDROOMStart **********************************

int CCTCManager::EDROOMStart()
{


	//***************** CEDROOMComponent::EDROOMStart*********

	CEDROOMComponent::EDROOMStart(); 	// Call to EDROOMStart method of CEDROOMComponent

	return 0;
}


//**************************************************************************
//***************************    PROTECTED METHODS *************************



//*****************************  EDROOMBehaviour ***************************



void CCTCManager::EDROOMBehaviour()
{

	edroomTopState.EDROOMInit();
	edroomTopState.EDROOMBehaviour();


		// *************** PUERTOS IRQ ********************

	RxTC.EndIRQHandlerTask();
}




//********************* ComponentIsFinished **********************************


#ifdef _EDROOM_SYSTEM_CLOSE

bool CCTCManager::EDROOMIsComponentFinished()
{


	return ( CEDROOMComponent::EDROOMIsComponentFinished());

}

#endif


//****************** EDROOMMemory::SetMemory *******************************

void CCTCManager::CEDROOMMemory::SetMemory(TEDROOMUInt32 numMessages_ ,
		CEDROOMMessage * MessagesMem_,
		bool * MessagesMemMarks_,
		TEDROOMUInt32 numberOfNodes_,
		CEDROOMQueue::CQueueNode * QueueNodesMem_,
		bool * QueueNodesMemMarks_)
{

		CEDROOMComponentMemory::SetMemory( numMessages_,MessagesMem_, MessagesMemMarks_,
			numberOfNodes_,QueueNodesMem_, QueueNodesMemMarks_, QueueHeads);


}

