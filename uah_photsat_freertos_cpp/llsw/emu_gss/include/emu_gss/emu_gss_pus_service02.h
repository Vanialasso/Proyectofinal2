/*
 * emu_sc_pus_service5.h
 *
 *  Created on: Jan 16, 2024
 *      Author: user
 */

#ifndef EMU_SC_PUS_SERVICE2_H_
#define EMU_SC_PUS_SERVICE2_H_

enum DeviceCommandOnOff_t { DeviceCommandOff, DeviceCommandOn};

class EmuGSS_TCProgram2_1: public EmuGSS_TCProgram {

	friend class EmuGSS_TCProgram19_1_Action_2_1;
	uint32_t mDeviceOnOffCommand;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram2_1(uint32_t uniTime2YK, const char *brief,
			enum DeviceCommandOnOff_t devicecommand);

	//Constructor for 19_1 event action program
	EmuGSS_TCProgram2_1(enum DeviceCommandOnOff_t devicecommand);

};


class EmuGSS_TCProgram2_1_BadAddress: public EmuGSS_TCProgram {

	uint32_t mDeviceOnOffBadAddress;

	virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor);

public:

	EmuGSS_TCProgram2_1_BadAddress(uint32_t uniTime2YK, const char *brief,
			uint32_t address);


};


#endif /* EMU_SC_PUS_SERVICE9_H_ */
