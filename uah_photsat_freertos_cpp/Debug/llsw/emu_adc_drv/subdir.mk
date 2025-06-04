################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../llsw/emu_adc_drv/emu_adc_drv.c 

C_DEPS += \
./llsw/emu_adc_drv/emu_adc_drv.d 

OBJS += \
./llsw/emu_adc_drv/emu_adc_drv.o 


# Each subdirectory must supply rules for building sources it contributes
llsw/emu_adc_drv/%.o: ../llsw/emu_adc_drv/%.c llsw/emu_adc_drv/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service3/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_sys_data_pool/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tm_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service1/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_pus_service17/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tm_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/obt_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_handler/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/asw_pus_services/asw_tc_accept_report/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_hw_timecode_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_adc_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/crc/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/edroomsl/edroombp/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_gss/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/ccsds_pus/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/tmtc_dyn_mem/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/service_libraries/serialize/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/emu_sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/sc_channel_drv/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/config/include" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr" -I"/home/opolo70/OBDH-workspace/asw_pus_INTA_freertos_project_2024_generado/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-llsw-2f-emu_adc_drv

clean-llsw-2f-emu_adc_drv:
	-$(RM) ./llsw/emu_adc_drv/emu_adc_drv.d ./llsw/emu_adc_drv/emu_adc_drv.o

.PHONY: clean-llsw-2f-emu_adc_drv

