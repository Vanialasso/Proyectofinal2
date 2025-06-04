#ifndef FCDTCMemDescriptorH
#define FCDTCMemDescriptorH

#include "public/basic_types.h"
#include "public/ccsds_pus.h"
#include "public/tmtc_dyn_mem.h"


class CDTCMemDescriptor{
	
	friend class CDTCHandler;

	friend void SC_Channel_GetNextTC(CDTCMemDescriptor &tc);

	protected:
	
	//!tc memory descriptor
		tc_mem_descriptor_t mTCMemDescriptor;

	public:

	//!returns true if the TC is commpleted after IRQ
	bool HandleIRQ();
	
};
#endif
