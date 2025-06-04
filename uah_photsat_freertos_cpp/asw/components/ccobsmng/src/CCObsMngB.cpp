#include <public/ccobsmng_iface_v1.h>

// ***********************************************************************

// class EDROOM_CTX_Top_0

// ***********************************************************************



	// CONSTRUCTORS***********************************************

CCObsMng::EDROOM_CTX_Top_0::EDROOM_CTX_Top_0(CCObsMng &act,
	 Pr_Time & EDROOMpVarVNextTimeOut ):

	EDROOMcomponent(act),
	Msg(EDROOMcomponent.Msg),
	MsgBack(EDROOMcomponent.MsgBack),
	ObservCtrl(EDROOMcomponent.ObservCtrl),
	ObservTimer(EDROOMcomponent.ObservTimer),
	AttCtrlTimer(EDROOMcomponent.AttCtrlTimer),
	CAttitudePeriod(0,100000),
	CImageInterval(0,500000),
	VNextTimeOut(EDROOMpVarVNextTimeOut)
{
}

CCObsMng::EDROOM_CTX_Top_0::EDROOM_CTX_Top_0(EDROOM_CTX_Top_0 &context):

	EDROOMcomponent(context.EDROOMcomponent),
	Msg(context.Msg),
	MsgBack(context.MsgBack),
	ObservCtrl(context.ObservCtrl),
	ObservTimer(context.ObservTimer),
	AttCtrlTimer(context.AttCtrlTimer),
	CAttitudePeriod(0,100000),
	CImageInterval(0,500000),
	VNextTimeOut(context.VNextTimeOut)
{

}

	// EDROOMSearchContextTrans********************************************

bool CCObsMng::EDROOM_CTX_Top_0::EDROOMSearchContextTrans(
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

void	CCObsMng::EDROOM_CTX_Top_0::FDoActtitudeCtrl()

{

 
pus_service129_do_attitude_ctrl();
 

}



void	CCObsMng::EDROOM_CTX_Top_0::FEndObservation()

{

//Take new absloute time reference 
VNextTimeOut.GetTime();
pus_service129_end_observation();

}



void	CCObsMng::EDROOM_CTX_Top_0::FExecCameraMngTC()

{
   //Handle Msg->data
  CDTCHandler & varSObservTC = *(CDTCHandler *)Msg->data;
	
	
		// Data access
	varSObservTC.ExecTC();

}



void	CCObsMng::EDROOM_CTX_Top_0::FInit()

{
   //Define absolute time
  Pr_Time time;
 
 
 
 time.GetTime();
 time+=CAttitudePeriod;
 
 VNextTimeOut=time;
 
pus_service129_init();
   //Program absolute timer 
   AttCtrlTimer.InformAt( time ); 
}



void	CCObsMng::EDROOM_CTX_Top_0::FProgAttitudeCtrl()

{
   //Define absolute time
  Pr_Time time;
 
	 
 
VNextTimeOut+=CAttitudePeriod;
 
time=VNextTimeOut;
   //Program absolute timer 
   AttCtrlTimer.InformAt( time ); 
}



void	CCObsMng::EDROOM_CTX_Top_0::FProgTakeImage()

{
   //Define interval
  Pr_Time interval;
	
 interval=CImageInterval;
   //Program relative timer 
   ObservTimer.InformIn( interval ); 
}



void	CCObsMng::EDROOM_CTX_Top_0::FTakeImage()

{

pus_service129_take_image();

}



void	CCObsMng::EDROOM_CTX_Top_0::FToObservation()

{

pus_service129_start_observation();

}



bool	CCObsMng::EDROOM_CTX_Top_0::GLastImage()

{

 
return pus_service129_is_last_image();

}



bool	CCObsMng::EDROOM_CTX_Top_0::GReadyToObservation()

{

return pus_service129_is_observation_ready();

}



	//********************************** Pools *************************************



// ***********************************************************************

// class EDROOM_SUB_Top_0

// ***********************************************************************



	// CONSTRUCTOR*************************************************

CCObsMng::EDROOM_SUB_Top_0::EDROOM_SUB_Top_0 (CCObsMng&act):
		EDROOM_CTX_Top_0(act,
			VNextTimeOut)
{

}

	//***************************** EDROOMBehaviour ********************************

void CCObsMng::EDROOM_SUB_Top_0::EDROOMBehaviour()
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
				FInit();
				//Next State is Standby
				edroomNextState = Standby;
				break;
			//Next Transition is ExecTC
			case (ExecTC):
				//Msg->Data Handling 
				FExecCameraMngTC();
				//Next State is Standby
				edroomNextState = Standby;
				break;
			//To Choice Point DoAttCtrl
			case (DoAttCtrl):

				//Execute Action 
				FDoActtitudeCtrl();
				//Evaluate Branch ToObservation
				if( GReadyToObservation() )
				{
					//Execute Action 
					FToObservation();

					//Branch taken is DoAttCtrl_ToObservation
					edroomCurrentTrans.localId =
						DoAttCtrl_ToObservation;

					//Next State is Observation
					edroomNextState = Observation;
				 } 
				//Default Branch ToStandBy
				else
				{
					//Execute Action 
					FProgAttitudeCtrl();

					//Branch taken is DoAttCtrl_ToStandBy
					edroomCurrentTrans.localId =
						DoAttCtrl_ToStandBy;

					//Next State is Standby
					edroomNextState = Standby;
				 } 
				break;
			//To Choice Point TakeImage
			case (TakeImage):

				//Execute Action 
				FTakeImage();
				//Evaluate Branch LastImage
				if( GLastImage() )
				{
					//Execute Actions 
					FEndObservation();
					FProgAttitudeCtrl();

					//Branch taken is TakeImage_LastImage
					edroomCurrentTrans.localId =
						TakeImage_LastImage;

					//Next State is Standby
					edroomNextState = Standby;
				 } 
				//Default Branch NextImage
				else
				{

					//Branch taken is TakeImage_NextImage
					edroomCurrentTrans.localId =
						TakeImage_NextImage;

					//Next State is Observation
					edroomNextState = Observation;
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

				//Go to the state Standby
			case (Standby):
				//Arrival to state Standby
				edroomCurrentTrans=EDROOMStandbyArrival();
				break;

				//Go to the state Observation
			case (Observation):
				//Execute Entry Action 
				FProgTakeImage();
				//Arrival to state Observation
				edroomCurrentTrans=EDROOMObservationArrival();
				break;

		}

		edroomCurrentState=edroomNextState;

	}while(Msg->signal != EDROOMSignalDestroy);

}



	// Context Init**********************************************

void CCObsMng::EDROOM_SUB_Top_0::EDROOMInit()
{

edroomCurrentState=I;

}



//	 ***********************************************************************

//	 SubState I

//	 ***********************************************************************



TEDROOMTransId CCObsMng::EDROOM_SUB_Top_0::EDROOMIArrival()
{

	TEDROOMTransId edroomCurrentTrans;

	//Next transition is  Init
	edroomCurrentTrans.localId = Init;
	edroomCurrentTrans.distanceToContext = 0;
	return(edroomCurrentTrans);

}



	// ***********************************************************************

	// Leaf SubState  Standby

	// ***********************************************************************



TEDROOMTransId CCObsMng::EDROOM_SUB_Top_0::EDROOMStandbyArrival()
{

	TEDROOMTransId edroomCurrentTrans;

	bool edroomValidMsg=false;

	do
	{

		EDROOMNewMessage ();

		switch(Msg->signal)
		{

			case (SObservTC): 

				 if (*Msg->GetPInterface() == ObservCtrl)
				{

					//Next transition is  ExecTC
					edroomCurrentTrans.localId= ExecTC;
					edroomCurrentTrans.distanceToContext = 0;
					edroomValidMsg=true;
				 }

				break;

			case (EDROOMSignalTimeout): 

				 if (*Msg->GetPInterface() == AttCtrlTimer)
				{

					//Next transition is  DoAttCtrl
					edroomCurrentTrans.localId = DoAttCtrl;
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



	// ***********************************************************************

	// Leaf SubState  Observation

	// ***********************************************************************



TEDROOMTransId CCObsMng::EDROOM_SUB_Top_0::EDROOMObservationArrival()
{

	TEDROOMTransId edroomCurrentTrans;

	bool edroomValidMsg=false;

	do
	{

		EDROOMNewMessage ();

		switch(Msg->signal)
		{

			case (EDROOMSignalTimeout): 

				 if (*Msg->GetPInterface() == ObservTimer)
				{

					//Next transition is  TakeImage
					edroomCurrentTrans.localId = TakeImage;
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



