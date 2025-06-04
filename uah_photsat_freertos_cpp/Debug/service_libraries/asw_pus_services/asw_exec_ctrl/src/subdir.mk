################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../service_libraries/asw_pus_services/asw_exec_ctrl/src/asw_exec_ctrl.c 

C_DEPS += \
./service_libraries/asw_pus_services/asw_exec_ctrl/src/asw_exec_ctrl.d 

OBJS += \
./service_libraries/asw_pus_services/asw_exec_ctrl/src/asw_exec_ctrl.o 


# Each subdirectory must supply rules for building sources it contributes
service_libraries/asw_pus_services/asw_exec_ctrl/src/%.o: ../service_libraries/asw_pus_services/asw_exec_ctrl/src/%.c service_libraries/asw_pus_services/asw_exec_ctrl/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tm_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service1/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service17/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tm_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/obt_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_accept_report/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_hw_timecode_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_adc_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/crc/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroombp/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_gss/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/ccsds_pus/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tmtc_dyn_mem/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/serialize/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-service_libraries-2f-asw_pus_services-2f-asw_exec_ctrl-2f-src

clean-service_libraries-2f-asw_pus_services-2f-asw_exec_ctrl-2f-src:
	-$(RM) ./service_libraries/asw_pus_services/asw_exec_ctrl/src/asw_exec_ctrl.d ./service_libraries/asw_pus_services/asw_exec_ctrl/src/asw_exec_ctrl.o

.PHONY: clean-service_libraries-2f-asw_pus_services-2f-asw_exec_ctrl-2f-src

