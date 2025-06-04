################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../components/cccontrolplant/src/CCControlPlantB.cpp \
../components/cccontrolplant/src/CCControlPlantS.cpp 

CPP_DEPS += \
./components/cccontrolplant/src/CCControlPlantB.d \
./components/cccontrolplant/src/CCControlPlantS.d 

OBJS += \
./components/cccontrolplant/src/CCControlPlantB.o \
./components/cccontrolplant/src/CCControlPlantS.o 


# Each subdirectory must supply rules for building sources it contributes
components/cccontrolplant/src/%.o: ../components/cccontrolplant/src/%.cpp components/cccontrolplant/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/dataclasses/CDControlParam/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/cccontrolplant/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/cchttpservertask/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/cckeyscantask/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/ccledtask/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/ccmonitor/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/ccrs232task/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/components/plantcontrolsystem/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroombp_swr/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/config/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroom_glue/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroomsl_swr/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/edroomsl_types_swr/include" -I"/home/opolo70/SEAR-workspace/edroom_freertos_plantcontrol_students/freertos_osswr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-components-2f-cccontrolplant-2f-src

clean-components-2f-cccontrolplant-2f-src:
	-$(RM) ./components/cccontrolplant/src/CCControlPlantB.d ./components/cccontrolplant/src/CCControlPlantB.o ./components/cccontrolplant/src/CCControlPlantS.d ./components/cccontrolplant/src/CCControlPlantS.o

.PHONY: clean-components-2f-cccontrolplant-2f-src

