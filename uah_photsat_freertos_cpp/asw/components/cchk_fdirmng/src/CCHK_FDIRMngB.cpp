#include <public/cchk_fdirmng_iface_v1.h>

// ***********************************************************************

// class EDROOM_CTX_Top_0

// ***********************************************************************



	// CONSTRUCTORS***********************************************

CCHK_FDIRMng::EDROOM_CTX_Top_0::EDROOM_CTX_Top_0(CCHK_FDIRMng &act,
	 Pr_Time & EDROOMpVarVNextTimeout,
	 CEDROOMPOOLCDEvAction & EDROOMpPoolCDEvAction ):

	EDROOMcomponent(act),
	Msg(EDROOMcomponent.Msg),
	MsgBack(EDROOMcomponent.MsgBack),
	HK_FDIRCtrl(EDROOMcomponent.HK_FDIRCtrl),
	HK_FDIRTimer(EDROOMcomponent.HK_FDIRTimer),
	VNextTimeout(EDROOMpVarVNextTimeout),
	EDROOMPoolCDEvAction(EDROOMpPoolCDEvAction)
{
}

CCHK_FDIRMng::EDROOM_CTX_Top_0::EDROOM_CTX_Top_0(EDROOM_CTX_Top_0 &context):

	EDROOMcomponent(context.EDROOMcomponent),
	Msg(context.Msg),
	MsgBack(context.MsgBack),
	HK_FDIRCtrl(context.HK_FDIRCtrl),
	HK_FDIRTimer(context.HK_FDIRTimer),
	VNextTimeout(context.VNextTimeout),
	EDROOMPoolCDEvAction(context.EDROOMPoolCDEvAction )
{

}

	// EDROOMSearchContextTrans********************************************

bool CCHK_FDIRMng::EDROOM_CTX_Top_0::EDROOMSearchContextTrans(
			TEDROOMTransId &edroomCurrentTrans)
			{

	bool edroomValidMsg=false; 

	 switch(Msg->signal)
	{

		 case ( EDROOMSignalDestroy ): 

			 edroomValidMsg=true;
			 edroomCurrentTrans.distanceToContext = 0 ;
			 edroomCurrentTrans.localId = -1 ;
			 break;

	}

	return(edroomValidMsg);

}

	// User-defined Functions   ****************************

void	CCHK_FDIRMng::EDROOM_CTX_Top_0::FDoHK_FDIR()

{
   //Define absolute time
  Pr_Time time;
 
VNextTimeout+= Pr_Time(1,0); // Add X sec + Y microsec 
time=VNextTimeout; 
 
pus_services_update_params();
pus_service4_update_all_stats();
pus_service3_do_HK();
pus_services_do_FDIR();
   //Program absolute timer 
   HK_FDIRTimer.InformAt( time ); 
}



void	CCHK_FDIRMng::EDROOM_CTX_Top_0::FExecHK_FDIR_TC()

{
   //Handle Msg->data
  CDTCHandler & varSHK_FDIR_TC = *(CDTCHandler *)Msg->data;
 
varSHK_FDIR_TC.ExecTC();

}



void	CCHK_FDIRMng::EDROOM_CTX_Top_0::FFwdEvAction()

{
   //Allocate data from pool
  CDEvAction * pSEvAction_Data = EDROOMPoolCDEvAction.AllocData();
	
		// Complete Data 
	
 
pSEvAction_Data->ExtractEvActionFromQueue();
   //Send message 
   HK_FDIRCtrl.send(SEvAction,pSEvAction_Data,&EDROOMPoolCDEvAction); 
}



void	CCHK_FDIRMng::EDROOM_CTX_Top_0::FInitHK_FDIR()

{
   //Define absolute time
  Pr_Time time;
 
time.GetTime(); // Get current monotonic time   
time+=Pr_Time(1,0); // Add X sec + Y microsec    
VNextTimeout=time;
 
 
   //Program absolute timer 
   HK_FDIRTimer.InformAt( time ); 
}



bool	CCHK_FDIRMng::EDROOM_CTX_Top_0::GPendingEvAction()

{

//return (false);
return (pus_service19_pending_ev_actions()); 
 

}



	//********************************** Pools *************************************

	//CEDROOMPOOLCDEvAction

CCHK_FDIRMng::EDROOM_CTX_Top_0::CEDROOMPOOLCDEvAction::CEDROOMPOOLCDEvAction(
			TEDROOMUInt32 elemCount,CDEvAction* pMem,bool * pMemMarks):
				CEDROOMProtectedMemoryPool(elemCount, pMem, pMemMarks,
					sizeof(CDEvAction))
{
}

CDEvAction *	CCHK_FDIRMng::EDROOM_CTX_Top_0::CEDROOMPOOLCDEvAction::AllocData()
{
	return(CDEvAction*)CEDROOMProtectedMemoryPool::AllocData();
}



// ***********************************************************************

// class EDROOM_SUB_Top_0

// ***********************************************************************



	// CONSTRUCTOR*************************************************

CCHK_FDIRMng::EDROOM_SUB_Top_0::EDROOM_SUB_Top_0 (CCHK_FDIRMng&act
	,CEDROOMMemory *pEDROOMMemory):
		EDROOM_CTX_Top_0(act,
			VNextTimeout,
			EDROOMPoolCDEvAction),
		EDROOMPoolCDEvAction(
			4, pEDROOMMemory->poolCDEvAction,
			pEDROOMMemory->poolMarkCDEvAction)
{

}

	//***************************** EDROOMBehaviour ********************************

void CCHK_FDIRMng::EDROOM_SUB_Top_0::EDROOMBehaviour()
{

	TEDROOMTransId edroomCurrentTrans;

	//Behaviour starts from Init State

	edroomCurrentTrans = EDROOMIArrival();

	do
	{

		//Take next transition

		switch(edroomCurrentTrans.localId)
		{

			//Next Transition is Init
			case (Init):
				//Execute Action 
				FInitHK_FDIR();
				//Next State is Ready
				edroomNextState = Ready;
				break;
			//Next Transition is ExecTC
			case (ExecTC):
				//Msg->Data Handling 
				FExecHK_FDIR_TC();
				//Next State is Ready
				edroomNextState = Ready;
				break;
			//To Choice Point DoHK_FDIR
			case (DoHK_FDIR):

				//Execute Action 
				FDoHK_FDIR();
				//Evaluate Branch PendingEvAction
				if( GPendingEvAction() )
				{
					//Send Asynchronous Message 
					FFwdEvAction();

					//Branch taken is DoHK_FDIR_PendingEvAction
					edroomCurrentTrans.localId =
						DoHK_FDIR_PendingEvAction;

					//Next State is Ready
					edroomNextState = Ready;
				 } 
				//Default Branch NoEvAction
				else
				{

					//Branch taken is DoHK_FDIR_NoEvAction
					edroomCurrentTrans.localId =
						DoHK_FDIR_NoEvAction;

					//Next State is Ready
					edroomNextState = Ready;
				 } 
				break;
		}

		//Entry into the Next State 
		switch(edroomNextState)
		{

				//Go to the state I
			case (I):
				//Arrival to state I
				edroomCurrentTrans=EDROOMIArrival();
				break;

				//Go to the state Ready
			case (Ready):
				//Arrival to state Ready
				edroomCurrentTrans=EDROOMReadyArrival();
				break;

		}

		edroomCurrentState=edroomNextState;

	}while(Msg->signal != EDROOMSignalDestroy);

}



	// Context Init**********************************************

void CCHK_FDIRMng::EDROOM_SUB_Top_0::EDROOMInit()
{

edroomCurrentState=I;

}



//	 ***********************************************************************

//	 SubState I

//	 ***********************************************************************



TEDROOMTransId CCHK_FDIRMng::EDROOM_SUB_Top_0::EDROOMIArrival()
{

	TEDROOMTransId edroomCurrentTrans;

	//Next transition is  Init
	edroomCurrentTrans.localId = Init;
	edroomCurrentTrans.distanceToContext = 0;
	return(edroomCurrentTrans);

}



	// ***********************************************************************

	// Leaf SubState  Ready

	// ***********************************************************************



TEDROOMTransId CCHK_FDIRMng::EDROOM_SUB_Top_0::EDROOMReadyArrival()
{

	TEDROOMTransId edroomCurrentTrans;

	bool edroomValidMsg=false;

	do
	{

		EDROOMNewMessage ();

		switch(Msg->signal)
		{

			case (SHK_FDIR_TC): 

				 if (*Msg->GetPInterface() == HK_FDIRCtrl)
				{

					//Next transition is  ExecTC
					edroomCurrentTrans.localId= ExecTC;
					edroomCurrentTrans.distanceToContext = 0;
					edroomValidMsg=true;
				 }

				break;

			case (EDROOMSignalTimeout): 

				 if (*Msg->GetPInterface() == HK_FDIRTimer)
				{

					//Next transition is  DoHK_FDIR
					edroomCurrentTrans.localId = DoHK_FDIR;
					edroomCurrentTrans.distanceToContext = 0 ;
					edroomValidMsg=true;
				 }

				break;

		};

		if (false == edroomValidMsg)
		{
			 edroomValidMsg = EDROOMSearchContextTrans(edroomCurrentTrans);

		}

	} while (false == edroomValidMsg);

	return(edroomCurrentTrans);

}



