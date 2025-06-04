################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../service_libraries/asw_pus_services/src/icuasw_pus_service1.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service12.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service128.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service17.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service19.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service2.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service20.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service3.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service5.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_service9.cpp \
../service_libraries/asw_pus_services/src/icuasw_pus_services.cpp 

CPP_DEPS += \
./service_libraries/asw_pus_services/src/icuasw_pus_service1.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service12.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service128.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service17.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service19.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service2.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service20.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service3.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service5.d \
./service_libraries/asw_pus_services/src/icuasw_pus_service9.d \
./service_libraries/asw_pus_services/src/icuasw_pus_services.d 

OBJS += \
./service_libraries/asw_pus_services/src/icuasw_pus_service1.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service12.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service128.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service17.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service19.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service2.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service20.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service3.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service5.o \
./service_libraries/asw_pus_services/src/icuasw_pus_service9.o \
./service_libraries/asw_pus_services/src/icuasw_pus_services.o 


# Each subdirectory must supply rules for building sources it contributes
service_libraries/asw_pus_services/src/%.o: ../service_libraries/asw_pus_services/src/%.cpp service_libraries/asw_pus_services/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service1/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTCAcceptReport/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_accept_report/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTCMemDescriptor/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_adc_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tc_rate_ctrl/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tc_queue_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_hw_timecode_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/crc/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTMHandler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_gss/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/ccsds_pus/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tmtc_dyn_mem/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/serialize/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/cchk_fdirmng/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/ccbkgtcexec/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/ccepdmanager/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/cctm_channelctrl/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/components/icuasw/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroombp/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroomsl/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroomsl_types/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDEventList/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix/MemMang" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDEvAction/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTCHandler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/dataclasses/CDTMList/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-service_libraries-2f-asw_pus_services-2f-src

clean-service_libraries-2f-asw_pus_services-2f-src:
	-$(RM) ./service_libraries/asw_pus_services/src/icuasw_pus_service1.d ./service_libraries/asw_pus_services/src/icuasw_pus_service1.o ./service_libraries/asw_pus_services/src/icuasw_pus_service12.d ./service_libraries/asw_pus_services/src/icuasw_pus_service12.o ./service_libraries/asw_pus_services/src/icuasw_pus_service128.d ./service_libraries/asw_pus_services/src/icuasw_pus_service128.o ./service_libraries/asw_pus_services/src/icuasw_pus_service17.d ./service_libraries/asw_pus_services/src/icuasw_pus_service17.o ./service_libraries/asw_pus_services/src/icuasw_pus_service19.d ./service_libraries/asw_pus_services/src/icuasw_pus_service19.o ./service_libraries/asw_pus_services/src/icuasw_pus_service2.d ./service_libraries/asw_pus_services/src/icuasw_pus_service2.o ./service_libraries/asw_pus_services/src/icuasw_pus_service20.d ./service_libraries/asw_pus_services/src/icuasw_pus_service20.o ./service_libraries/asw_pus_services/src/icuasw_pus_service3.d ./service_libraries/asw_pus_services/src/icuasw_pus_service3.o ./service_libraries/asw_pus_services/src/icuasw_pus_service5.d ./service_libraries/asw_pus_services/src/icuasw_pus_service5.o ./service_libraries/asw_pus_services/src/icuasw_pus_service9.d ./service_libraries/asw_pus_services/src/icuasw_pus_service9.o ./service_libraries/asw_pus_services/src/icuasw_pus_services.d ./service_libraries/asw_pus_services/src/icuasw_pus_services.o

.PHONY: clean-service_libraries-2f-asw_pus_services-2f-src

