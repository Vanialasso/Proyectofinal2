/*
 * emu_gss_tc_program.h
 *
 *  Created on: Jan 9, 2024
 *      Author: opolo70
 */

#ifndef LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_TC_PROGRAM_H_
#define LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_TC_PROGRAM_H_

#include "public/tmtc_dyn_mem.h"

class EmuGSS_TCProgram{

	friend void EmuGSS_SendProgrammedTCs();
	friend void EmuGSS_RxCurrentTC(CDTCMemDescriptor * pTCDescriptor);

protected:
  //tc_mem_descriptor_t tc_descriptor;
  uint32_t OBTSecs;
  EmuGSS_TCProgram *pmNext;

  uint8_t mType;
  uint8_t mSubtype;
  uint16_t mAppDataLength;

  uint8_t mACKFlags;

  const char * mBrief;

  static uint32_t sTCSeqCounter;

  static EmuGSS_TCProgram *sFirstTCProgram;
  static EmuGSS_TCProgram *sCurrentTCProgram;

  byte_t *pmAppData;

  static void NewProgram(EmuGSS_TCProgram *);

  EmuGSS_TCProgram(uint32_t obt_secn,
            uint8_t type,
            uint8_t subtype,
			uint16_t appDataLength,
            const char * brief);
public:
  static bool SetCurrentTCProgramed(uint32_t obt_seconds);

  uint8_t GetType(){return mType;}
  uint8_t GetSubtype(){return mSubtype;}
  const char * GetBrief(){return mBrief;}

private:
  void BuildTCPackHeader(tc_mem_descriptor_t &tc_descriptor);
  void BuildTCDFHeader(tc_mem_descriptor_t &tc_descriptor);
  virtual void BuildTCAppData(tc_mem_descriptor_t &tc_descriptor)=0;

protected:

  static EmuGSS_TCProgram * GetCurrentTCProgramed();

  void BuildTC(tc_mem_descriptor_t &tc_descriptor);

  void DisableAcceptACK();
  void DisableExecStartACK();
  void DisableExecProgACK();
  void DisableExecCompletionACK();


  void SetNextUInt8(uint8_t data);

  void SetNextUInt16(uint16_t data);

  void SetNextUInt32(uint32_t data);

  void SetNextUInt64(uint64_t data);

  void SetNextInt8(int8_t data);

  void SetNextInt16(int16_t data);

  void SetNextInt32(int32_t data);

  void SetNextInt64(int64_t data);

  void SetNextFloat(float & data);

};




#endif /* LLSW_EMU_GSS_INCLUDE_EMU_GSS_EMU_GSS_TC_PROGRAM_H_ */
