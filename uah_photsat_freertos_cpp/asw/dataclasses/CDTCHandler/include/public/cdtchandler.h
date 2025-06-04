#ifndef FCDTCHandlerH
#define FCDTCHandlerH

#include "public/config.h"
#include "public/basic_types.h"
#include "public/ccsds_pus.h"
#include "public/tmtc_dyn_mem.h"
#include "public/pus_tc_handler.h"

#include "public/cdtcmemdescriptor_iface_v1.h"
#include "public/cdtcacceptreport_iface_v1.h"
#include "public/cdtcexecctrl_iface_v1.h"


class CDTCHandler {

	friend class CDEvAction;


protected:

	tc_handler_t mTCHandler;

public:

	//!Constructor
	CDTCHandler();

	//!Build From Descriptor
	void BuildFromDescriptor(CDTCMemDescriptor &descriptor);

	//!Do TC Acceptation
	CDTCAcceptReport DoAcceptation();

	//!Mng TC Rejection
	void MngTCRejection(CDTCAcceptReport & acceptReport);

	//!Mng TC Accetation
	void MngTCAcceptation();

	//!Set Execution Control 
	CDTCExecCtrl GetExecCtrl();

	//!Exec the telecommand
	void ExecTC();


};

#endif
