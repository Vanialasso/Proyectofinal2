################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../asw/components/ccepdmanager/src/CCEPDManagerB.cpp \
../asw/components/ccepdmanager/src/CCEPDManagerS.cpp 

CPP_DEPS += \
./asw/components/ccepdmanager/src/CCEPDManagerB.d \
./asw/components/ccepdmanager/src/CCEPDManagerS.d 

OBJS += \
./asw/components/ccepdmanager/src/CCEPDManagerB.o \
./asw/components/ccepdmanager/src/CCEPDManagerS.o 


# Each subdirectory must supply rules for building sources it contributes
asw/components/ccepdmanager/src/%.o: ../asw/components/ccepdmanager/src/%.cpp asw/components/ccepdmanager/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service1/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTCAcceptReport/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_accept_report/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTCMemDescriptor/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_adc_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tc_rate_ctrl/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tc_queue_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_hw_timecode_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/crc/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTMHandler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_gss/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/ccsds_pus/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tmtc_dyn_mem/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/serialize/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/cchk_fdirmng/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/ccbkgtcexec/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/ccepdmanager/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/cctm_channelctrl/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/icuasw/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroombp/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroomsl/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroomsl_types/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDEventList/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix/MemMang" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDEvAction/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTCHandler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTMList/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-asw-2f-components-2f-ccepdmanager-2f-src

clean-asw-2f-components-2f-ccepdmanager-2f-src:
	-$(RM) ./asw/components/ccepdmanager/src/CCEPDManagerB.d ./asw/components/ccepdmanager/src/CCEPDManagerB.o ./asw/components/ccepdmanager/src/CCEPDManagerS.d ./asw/components/ccepdmanager/src/CCEPDManagerS.o

.PHONY: clean-asw-2f-components-2f-ccepdmanager-2f-src

