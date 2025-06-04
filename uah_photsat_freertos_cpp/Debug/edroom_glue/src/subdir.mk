################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../edroom_glue/src/edroomdeployment.cpp 

CPP_DEPS += \
./edroom_glue/src/edroomdeployment.d 

OBJS += \
./edroom_glue/src/edroomdeployment.o 


# Each subdirectory must supply rules for building sources it contributes
edroom_glue/src/%.o: ../edroom_glue/src/%.cpp edroom_glue/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/dataclasses/CDControlParam/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/cccontrolplant/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/cchttpservertask/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/cckeyscantask/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/ccledtask/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/ccmonitor/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/ccrs232task/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/plantcontrolsystem/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroombp_swr/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/config/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroom_glue/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroomsl_swr/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroomsl_types_swr/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-edroom_glue-2f-src

clean-edroom_glue-2f-src:
	-$(RM) ./edroom_glue/src/edroomdeployment.d ./edroom_glue/src/edroomdeployment.o

.PHONY: clean-edroom_glue-2f-src

