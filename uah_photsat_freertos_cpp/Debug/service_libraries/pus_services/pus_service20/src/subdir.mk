################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../service_libraries/pus_services/pus_service20/src/aux_pus_service20_tx_tm.c \
../service_libraries/pus_services/pus_service20/src/aux_pus_sevice20_exec_tc.c \
../service_libraries/pus_services/pus_service20/src/pus_service20.c 

C_DEPS += \
./service_libraries/pus_services/pus_service20/src/aux_pus_service20_tx_tm.d \
./service_libraries/pus_services/pus_service20/src/aux_pus_sevice20_exec_tc.d \
./service_libraries/pus_services/pus_service20/src/pus_service20.d 

OBJS += \
./service_libraries/pus_services/pus_service20/src/aux_pus_service20_tx_tm.o \
./service_libraries/pus_services/pus_service20/src/aux_pus_sevice20_exec_tc.o \
./service_libraries/pus_services/pus_service20/src/pus_service20.o 


# Each subdirectory must supply rules for building sources it contributes
service_libraries/pus_services/pus_service20/src/%.o: ../service_libraries/pus_services/pus_service20/src/%.c service_libraries/pus_services/pus_service20/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/attitude_pid_ctrl/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/tc_rate_ctrl/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service129/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/emu_sat_attitude/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service09/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service128/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service04/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/emu_watchdog_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service06/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/device_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service02/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service19/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service12/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service20/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service05/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service03/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_sys_data_pool/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_tm_handler/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service01/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_service17/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_tm_handler/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/obt_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_tc_handler/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/pus_services/pus_tc_accept_report/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/emu_hw_timecode_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/emu_adc_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/asw/edroom_glue/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/crc/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/edroomsl/edroombp/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/emu_gss/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/ccsds_pus/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/tmtc_dyn_mem/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/service_libraries/serialize/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/config/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/emu_sc_channel_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/sc_channel_drv/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/config/include" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/freertos_osswr" -I"/home/opolo70/OBDH_2025-workspace/uah_photsat_freertos_cpp/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-service_libraries-2f-pus_services-2f-pus_service20-2f-src

clean-service_libraries-2f-pus_services-2f-pus_service20-2f-src:
	-$(RM) ./service_libraries/pus_services/pus_service20/src/aux_pus_service20_tx_tm.d ./service_libraries/pus_services/pus_service20/src/aux_pus_service20_tx_tm.o ./service_libraries/pus_services/pus_service20/src/aux_pus_sevice20_exec_tc.d ./service_libraries/pus_services/pus_service20/src/aux_pus_sevice20_exec_tc.o ./service_libraries/pus_services/pus_service20/src/pus_service20.d ./service_libraries/pus_services/pus_service20/src/pus_service20.o

.PHONY: clean-service_libraries-2f-pus_services-2f-pus_service20-2f-src

