################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../service_libraries/pus_services/pus_service3/src/aux_pus_service3_exec_tc.c \
../service_libraries/pus_services/pus_service3/src/aux_pus_service3_tx_tm.c \
../service_libraries/pus_services/pus_service3/src/aux_pus_service3_utils.c \
../service_libraries/pus_services/pus_service3/src/pus_service3.c 

C_DEPS += \
./service_libraries/pus_services/pus_service3/src/aux_pus_service3_exec_tc.d \
./service_libraries/pus_services/pus_service3/src/aux_pus_service3_tx_tm.d \
./service_libraries/pus_services/pus_service3/src/aux_pus_service3_utils.d \
./service_libraries/pus_services/pus_service3/src/pus_service3.d 

OBJS += \
./service_libraries/pus_services/pus_service3/src/aux_pus_service3_exec_tc.o \
./service_libraries/pus_services/pus_service3/src/aux_pus_service3_tx_tm.o \
./service_libraries/pus_services/pus_service3/src/aux_pus_service3_utils.o \
./service_libraries/pus_services/pus_service3/src/pus_service3.o 


# Each subdirectory must supply rules for building sources it contributes
service_libraries/pus_services/pus_service3/src/%.o: ../service_libraries/pus_services/pus_service3/src/%.c service_libraries/pus_services/pus_service3/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/freertos_osswr/Source/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service6/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service4/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/device_drv/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service2/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service19/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service12/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service20/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service5/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service5/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service3/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_sys_data_pool/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_tm_handler/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service1/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service17/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_tm_handler/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/obt_drv/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_tc_handler/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_service1/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/pus_services/pus_tc_accept_report/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/emu_hw_timecode_drv/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/emu_adc_drv/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/asw/edroom_glue/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/crc/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/edroomsl/edroombp/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/emu_gss/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/ccsds_pus/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/tmtc_dyn_mem/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/service_libraries/serialize/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/config/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/emu_sc_channel_drv/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/sc_channel_drv/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/config/include" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/freertos_osswr" -I"/home/opolo70/INTA-workspace/asw_pus_freertos_st19_todo/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-service_libraries-2f-pus_services-2f-pus_service3-2f-src

clean-service_libraries-2f-pus_services-2f-pus_service3-2f-src:
	-$(RM) ./service_libraries/pus_services/pus_service3/src/aux_pus_service3_exec_tc.d ./service_libraries/pus_services/pus_service3/src/aux_pus_service3_exec_tc.o ./service_libraries/pus_services/pus_service3/src/aux_pus_service3_tx_tm.d ./service_libraries/pus_services/pus_service3/src/aux_pus_service3_tx_tm.o ./service_libraries/pus_services/pus_service3/src/aux_pus_service3_utils.d ./service_libraries/pus_services/pus_service3/src/aux_pus_service3_utils.o ./service_libraries/pus_services/pus_service3/src/pus_service3.d ./service_libraries/pus_services/pus_service3/src/pus_service3.o

.PHONY: clean-service_libraries-2f-pus_services-2f-pus_service3-2f-src

