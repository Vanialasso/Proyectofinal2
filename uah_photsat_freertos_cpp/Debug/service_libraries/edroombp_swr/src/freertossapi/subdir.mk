################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../service_libraries/edroombp_swr/src/freertossapi/edroombp.cpp 

C_SRCS += \
../service_libraries/edroombp_swr/src/freertossapi/timespec.c 

CPP_DEPS += \
./service_libraries/edroombp_swr/src/freertossapi/edroombp.d 

C_DEPS += \
./service_libraries/edroombp_swr/src/freertossapi/timespec.d 

OBJS += \
./service_libraries/edroombp_swr/src/freertossapi/edroombp.o \
./service_libraries/edroombp_swr/src/freertossapi/timespec.o 


# Each subdirectory must supply rules for building sources it contributes
service_libraries/edroombp_swr/src/freertossapi/%.o: ../service_libraries/edroombp_swr/src/freertossapi/%.cpp service_libraries/edroombp_swr/src/freertossapi/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/dataclasses/CDControlParam/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/cccontrolplant/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/cchttpservertask/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/cckeyscantask/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/ccledtask/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/ccmonitor/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/ccrs232task/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/plantcontrolsystem/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroombp_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/config/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroomsl_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroomsl_types_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

service_libraries/edroombp_swr/src/freertossapi/%.o: ../service_libraries/edroombp_swr/src/freertossapi/%.c service_libraries/edroombp_swr/src/freertossapi/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/config/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroombp_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-service_libraries-2f-edroombp_swr-2f-src-2f-freertossapi

clean-service_libraries-2f-edroombp_swr-2f-src-2f-freertossapi:
	-$(RM) ./service_libraries/edroombp_swr/src/freertossapi/edroombp.d ./service_libraries/edroombp_swr/src/freertossapi/edroombp.o ./service_libraries/edroombp_swr/src/freertossapi/timespec.d ./service_libraries/edroombp_swr/src/freertossapi/timespec.o

.PHONY: clean-service_libraries-2f-edroombp_swr-2f-src-2f-freertossapi

