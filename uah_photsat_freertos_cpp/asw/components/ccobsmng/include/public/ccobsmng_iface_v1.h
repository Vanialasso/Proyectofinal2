#ifndef CCOBSMNG_H_
#define CCOBSMNG_H_

//******************************************************************************
// EDROOM Service Library

#define _EDROOM_IS_EMBEDDED_

#include <public/edroomsl_iface_v1.h>

//******************************************************************************
// Data Classes

#include <public/cdtchandler_iface_v1.h>


//******************************************************************************
// Required software interfaces

#include <public/pus_services_iface_v1.h>


/**
 * \class   CCObsMng
 *
 */
class CCObsMng: public CEDROOMComponent {

public:

	/**
	 * \enum TEDROOMCCObsMngSignal
	 * \brief Component Signal Enum Type
	 *
	 */
	 enum TEDROOMCCObsMngSignal { EDROOMSignalTimeout, 
							EDROOMSignalDestroy, 
							SObservTC };

	/**
	 * \class CCObsMng::CEDROOMMemory
	 * \brief Component Memory
	 *
	 */
	class CEDROOMMemory: public CEDROOMComponentMemory{

		private:

			//!Array of Message Queue Heads, one for each priority
			CEDROOMQueue::CQueueHead QueueHeads[EDROOMprioMINIMUM+1];

			//************ Component Timing Service Memory************

			//!Component Timing Service Timer Info Memory
			CEDROOMTimerInfo TimerInf[5];
			//!Component Timing Service Timer Info Marks Memory
			bool TimerInfMarks[5];
			//!Component Timing Service TimeOut Messages Memory
			CEDROOMTimeOutMessage TimeOutMsgs[5];
			//!Component Timing Service TimeOut Messages Marks Memory
			bool TimeOutMsgsMarks[5];

		public: 

			//!Component Timing Service Memory Object
			CEDROOMTimingMemory TimingMemory;



			/** \brief This function is used for setting the Component Memory
			 * 
			 * \param numberOfMsgs number of messages that the component can store
			 * \param msgsMemory memory for the messages that the component can store
			 * \param msgsMemoryMarks memory marks for the messages that the component can store
			 * \param numberOfNodes number of nodes that the component needs
			 * \param queueNodesMemory memory for the component message queues 
			 * \param queueNodesMemoryMarks memory marks for the component message queues 
			 */
			void SetMemory(TEDROOMUInt32 numberOfMsgs
						, CEDROOMMessage * msgsMemory
						, bool * msgsMemoryMarks
						, TEDROOMUInt32 numberOfNodes
						, CEDROOMQueue::CQueueNode * queueNodesMemory
						, bool * queueNodesMemoryMarks);

	};


	// ********************************************************************
	//******************  Component Communication Ports *******************
	// ********************************************************************

	//! ObservCtrl Component Port
	CEDROOMInterface	ObservCtrl;


	// ********************************************************************
	// ********************  Timing Service Interface *********************
	// ********************************************************************

	//! Timing Service Access Point. It is common to all timing ports.
	CEDROOMTimingSAP	 EDROOMtimingSAP;


	//! ObservTimer Timing Port
	CEDROOMTimingInterface	ObservTimer;
	//! AttCtrlTimer Timing Port
	CEDROOMTimingInterface	AttCtrlTimer;




	// *************************************************************************
	// **************************** Frame Service Methods	********************
	// *************************************************************************



	//! Constructor
	CCObsMng(TEDROOMComponentID id, TEDROOMUInt32 roomNumMaxMens, TEDROOMPriority roomtaskPrio, 
		TEDROOMStackSizeType roomStack, CEDROOMMemory * pActorMemory );




	//! Component Configuration 
	 int EDROOMConfig();


	//! Component Start 
	 int EDROOMStart();




	#ifdef _EDROOM_SYSTEM_CLOSE

	//! Method that returns true if component is finished 
	bool EDROOMIsComponentFinished();


	#endif

protected:



	//! Component behaviour 
	virtual void EDROOMBehaviour();




public:

	// *****************************	Behaviour  *************************

	// ***********************************************************************

	// class EDROOM_CTX_Top_0

	// ***********************************************************************



	class EDROOM_CTX_Top_0 {

	protected:

	/**
	 * \enum TEDROOMCCObsMngSignal
	 * \brief Component Signal Enum Type
	 *
	 */
	enum TEDROOMCCObsMngSignal { EDROOMSignalTimeout,
		EDROOMSignalDestroy,
		SObservTC };


		friend class CCObsMng;

		//!component reference
		CCObsMng &EDROOMcomponent;

		//!Current message pointer reference
		CEDROOMMessage * &Msg;

		//!Synchronous message back pointer reference
		CEDROOMMessage * &MsgBack;

		//!Component ports
		CEDROOMInterface & ObservCtrl;
		CEDROOMTimingInterface & ObservTimer;
		CEDROOMTimingInterface & AttCtrlTimer;


		//! State Identifiers
		enum TEDROOMStateID{I,
			Standby,
			Observation};

		//!Transition Identifiers
		enum TEDROOMTransitionID{Init,
			ExecTC,
			DoAttCtrl,
			DoAttCtrl_ToObservation,
			DoAttCtrl_ToStandBy,
			TakeImage,
			TakeImage_LastImage,
			TakeImage_NextImage,
			EDROOMMemoryTrans };

		//!Constants
		const Pr_Time CAttitudePeriod;
		const Pr_Time CImageInterval;


		//!Variables
		Pr_Time &VNextTimeOut;




		//!Constructor
		EDROOM_CTX_Top_0 (CCObsMng &act,
				Pr_Time & EDROOMpVarVNextTimeOut );

		//!Copy constructor
		EDROOM_CTX_Top_0 (EDROOM_CTX_Top_0 &context);

	public:

		/**
		 * \brief Search Context transition 
		 * \param edroomCurrentTrans returns the context transition 

		 * \return true if context transition is found 

		 */
		bool EDROOMSearchContextTrans(TEDROOMTransId &edroomCurrentTrans);

		//! \brief Get new message from the Queue

		void EDROOMNewMessage()
		{ EDROOMcomponent.EDROOMNewMsg(); }

		/**
		 * \brief Get and Clear the Error Flags 
		 * \return the error flags  

		 */
		 TEDROOMUInt32 GetAndClearErrorFlags(){ return EDROOMcomponent.GetAndClearErrorFlags();}

		// User-defined Functions

		/**
		 * \brief  
		 */
		void	FDoActtitudeCtrl();

		/**
		 * \brief  
		 */
		void	FEndObservation();

		/**
		 * \brief  
		 */
		void	FExecCameraMngTC();

		/**
		 * \brief  
		 */
		void	FInit();

		/**
		 * \brief  
		 */
		void	FProgAttitudeCtrl();

		/**
		 * \brief  
		 */
		void	FProgTakeImage();

		/**
		 * \brief  
		 */
		void	FTakeImage();

		/**
		 * \brief  
		 */
		void	FToObservation();

		/**
		 * \brief  
		 */
		bool	GLastImage();

		/**
		 * \brief  
		 */
		bool	GReadyToObservation();

	};

	// ***********************************************************************

	// class EDROOM_SUB_Top_0

	// ***********************************************************************



	class EDROOM_SUB_Top_0 : public EDROOM_CTX_Top_0 {

	protected:



		//!current state identifier
		EDROOM_CTX_Top_0::TEDROOMStateID edroomCurrentState;

		//!next state identifier
		EDROOM_CTX_Top_0::TEDROOMStateID edroomNextState;

		//!Variables
		Pr_Time VNextTimeOut;




	public:

		//! Constructor
		EDROOM_SUB_Top_0 (CCObsMng &act );


		//! Top Context Behaviour 

		void EDROOMBehaviour();

		//!Top Context Init
		void EDROOMInit();

		//! Initial substate arrival
		TEDROOMTransId EDROOMIArrival();

		// ***********************************************************************

		// Leaf SubState Standby

		// ***********************************************************************



		TEDROOMTransId EDROOMStandbyArrival();

		// ***********************************************************************

		// Leaf SubState Observation

		// ***********************************************************************



		TEDROOMTransId EDROOMObservationArrival();

	};

protected:

	//!Top State
	EDROOM_SUB_Top_0 edroomTopState;



};
#endif
