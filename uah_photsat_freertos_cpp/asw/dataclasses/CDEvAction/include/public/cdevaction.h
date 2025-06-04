#ifndef FCDEvActionH
#define FCDEvActionH

#include "public/basic_types.h"
#include "public/cdtchandler_iface_v1.h"


class CDEvAction{

	CDTCHandler mTCHandler;

	public:

	 /* \brief get the TCHandler associated to the action
	   * \returns the TCHandler associated to the action
	  */

	CDTCHandler GetActionTCHandler(){

		return mTCHandler;
	}

	 /* \brief set Action TCHandler associated to the action
	  */

	void SetActionTC(CDTCHandler & actionTC){

		mTCHandler=actionTC;
	}

	 /* \brief Extract Ev Action From Queue
	  */
	void ExtractEvActionFromQueue();

	CDEvAction(){
		mTCHandler.mTCHandler.raw_tc_mem_descriptor.p_tc_bytes=NULL;
		mTCHandler.mTCHandler.raw_tc_mem_descriptor.tc_num_bytes=0;
	}



	protected:
	
	private:
	
};
#endif
