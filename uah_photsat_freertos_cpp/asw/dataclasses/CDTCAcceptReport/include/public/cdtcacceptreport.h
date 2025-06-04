#ifndef FCDTCAcceptReportH
#define FCDTCAcceptReportH

#include "public/pus_tc_accept_report.h"

class CDTCAcceptReport{
	
	friend class CDTCHandler;

	//tc accept report
	tc_accept_report_t mAcceptReport;

	public:

	//!Constructior	
	CDTCAcceptReport(){
		mAcceptReport.accept_status=TCAcceptationNotProcessed;
	}

	//!return true if accepted
	bool IsAccepted(){
		return (TCAcceptationOK== mAcceptReport.accept_status);

	}

	protected:
	
	private:
	
};
#endif
