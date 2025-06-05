#include <public/obsmng_iface_v1.h>

// ***********************************************************************

// class EDROOM_CTX_Top_0

// ***********************************************************************



	// CONSTRUCTORS***********************************************

ObsMng::EDROOM_CTX_Top_0::EDROOM_CTX_Top_0(ObsMng &act,
	 Pr_Time & EDROOMpVarVNextTimeout ):

	EDROOMcomponent(act),
	Msg(EDROOMcomponent.Msg),
	MsgBack(EDROOMcomponent.MsgBack),
	Obsmng(EDROOMcomponent.Obsmng),
	ObservTimer(EDROOMcomponent.ObservTimer),
	AttCtrlTimer(EDROOMcomponent.AttCtrlTimer),
	CImageInterval(0,500000),
	VNextTimeout(EDROOMpVarVNextTimeout)
{
}

ObsMng::EDROOM_CTX_Top_0::EDROOM_CTX_Top_0(EDROOM_CTX_Top_0 &context):

	EDROOMcomponent(context.EDROOMcomponent),
	Msg(context.Msg),
	MsgBack(context.MsgBack),
	Obsmng(context.Obsmng),
	ObservTimer(context.ObservTimer),
	AttCtrlTimer(context.AttCtrlTimer),
	CImageInterval(0,500000),
	VNextTimeout(context.VNextTimeout)
{

}

	// EDROOMSearchContextTrans********************************************

bool ObsMng::EDROOM_CTX_Top_0::EDROOMSearchContextTrans(
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

void	ObsMng::EDROOM_CTX_Top_0::FDoAttitudeCtrl()

{

 pus_service129_do_attitude_ctrl() 

}



void	ObsMng::EDROOM_CTX_Top_0::FEndObservation()

{

VNextTimeOut.GetTime();
 pus_service129_end_observation();

}



void	ObsMng::EDROOM_CTX_Top_0::FExecObsMng_TC()

{


}



void	ObsMng::EDROOM_CTX_Top_0::Finit()

{


}



void	ObsMng::EDROOM_CTX_Top_0::FProgAttitudeCtrl()

{
   //Define absolute time
  Pr_Time time;
	 
	//Timing Service useful methods
	 
	//time.GetTime(); // Get current monotonic time
	//time.Add(X,Y); // Add X sec + Y microsec
   //Program absolute timer 
   ObservTimer.InformAt( time ); 
}



void	ObsMng::EDROOM_CTX_Top_0::FProgTakeImage()

{
   //Define absolute time
  Pr_Time time;
	 
	//Timing Service useful methods
	 
	//time.GetTime(); // Get current monotonic time
	//time.Add(X,Y); // Add X sec + Y microsec
interval=CImageInterval;
   //Program absolute timer 
   ObservTimer.InformAt( time ); 
}



void	ObsMng::EDROOM_CTX_Top_0::FTakeImage()

{

 pus_service129_take_image()

}



void	ObsMng::EDROOM_CTX_Top_0::FToObservation()

{

pus_service129_is_observation_ready()

}



bool	ObsMng::EDROOM_CTX_Top_0::GLastImage()

{

return pus_service129_is_last_image()

}



bool	ObsMng::EDROOM_CTX_Top_0::GReadyToObservation()

{

 return pus_service129_is_observation_ready()

}



	//********************************** Pools *************************************



// ***********************************************************************

// class EDROOM_SUB_Top_0

// ***********************************************************************



	// CONSTRUCTOR*************************************************

ObsMng::EDROOM_SUB_Top_0::EDROOM_SUB_Top_0 (ObsMng&act):
		EDROOM_CTX_Top_0(act,
			VNextTimeout)
{

}

	//***************************** EDROOMBehaviour ********************************

void ObsMng::EDROOM_SUB_Top_0::EDROOMBehaviour()
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
				Finit();
				//Next State is Standby
				edroomNextState = Standby;
				break;
			//To Choice Point DoAttitudeCtrl
			case (DoAttitudeCtrl):

				//Execute Action 
				FDoAttitudeCtrl();
				//Evaluate Branch ToObservation()
				if( GReadyToObservation() )
				{
					//Execute Action 
					FToObservation();

					//Branch taken is DoAttitudeCtrl_ToObservation()
					edroomCurrentTrans.localId =
						DoAttitudeCtrl_ToObservation();

					//Next State is Observation
					edroomNextState = Observation;
				 } 
				//Default Branch ProgAttitudeCtrl
				else
				{
					//Execute Action 
					FProgAttitudeCtrl();

					//Branch taken is DoAttitudeCtrl_ProgAttitudeCtrl
					edroomCurrentTrans.localId =
						DoAttitudeCtrl_ProgAttitudeCtrl;

					//Next State is Standby
					edroomNextState = Standby;
				 } 
				break;
			//To Choice Point TakeImage()
			case (TakeImage()):

				//Execute Action 
				FTakeImage();
				//Evaluate Branch ObservationProg
				if( GLastImage() )
				{
					//Execute Actions 
					FEndObservation();
					FProgAttitudeCtrl();

					//Branch taken is TakeImage()_ObservationProg
					edroomCurrentTrans.localId =
						TakeImage()_ObservationProg;

					//Next State is Standby
					edroomNextState = Standby;
				 } 
				//Default Branch ProgTakeImage()
				else
				{
					//Execute Action 
					FProgTakeImage();

					//Branch taken is TakeImage()_ProgTakeImage()
					edroomCurrentTrans.localId =
						TakeImage()_ProgTakeImage();

					//Next State is Observation
					edroomNextState = Observation;
				 } 
				break;
			//Next Transition is ExecObsMngTC
			case (ExecObsMngTC):
				//Execute Action 
				FExecObsMng_TC();
				//Next State is Standby
				edroomNextState = Standby;
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

void ObsMng::EDROOM_SUB_Top_0::EDROOMInit()
{

edroomCurrentState=I;

}



//	 ***********************************************************************

//	 SubState I

//	 ***********************************************************************



TEDROOMTransId ObsMng::EDROOM_SUB_Top_0::EDROOMIArrival()
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



TEDROOMTransId ObsMng::EDROOM_SUB_Top_0::EDROOMStandbyArrival()
{

	TEDROOMTransId edroomCurrentTrans;

	bool edroomValidMsg=false;

	do
	{

		EDROOMNewMessage ();

		switch(Msg->signal)
		{

			case (EDROOMSignalTimeout): 

				 if (*Msg->GetPInterface() == AttCtrlTimer)
				{

					//Next transition is  DoAttitudeCtrl
					edroomCurrentTrans.localId = DoAttitudeCtrl;
					edroomCurrentTrans.distanceToContext = 0 ;
					edroomValidMsg=true;
				 }

				break;

			case (SObsMng_TC): 

				 if (*Msg->GetPInterface() == Obsmng)
				{

					//Next transition is  ExecObsMngTC
					edroomCurrentTrans.localId= ExecObsMngTC;
					edroomCurrentTrans.distanceToContext = 0;
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



TEDROOMTransId ObsMng::EDROOM_SUB_Top_0::EDROOMObservationArrival()
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

					//Next transition is  TakeImage()
					edroomCurrentTrans.localId = TakeImage();
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



