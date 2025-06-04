################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../asw/components/plantcontrolsystem/src/PlantControlSystemB.cpp \
../asw/components/plantcontrolsystem/src/PlantControlSystemS.cpp 

CPP_DEPS += \
./asw/components/plantcontrolsystem/src/PlantControlSystemB.d \
./asw/components/plantcontrolsystem/src/PlantControlSystemS.d 

OBJS += \
./asw/components/plantcontrolsystem/src/PlantControlSystemB.o \
./asw/components/plantcontrolsystem/src/PlantControlSystemS.o 


# Each subdirectory must supply rules for building sources it contributes
asw/components/plantcontrolsystem/src/%.o: ../asw/components/plantcontrolsystem/src/%.cpp asw/components/plantcontrolsystem/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/dataclasses/CDControlParam/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/cccontrolplant/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/cchttpservertask/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/cckeyscantask/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/ccledtask/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/ccmonitor/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/ccrs232task/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/components/plantcontrolsystem/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroombp_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/config/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/asw/edroom_glue/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroomsl_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/service_libraries/edroomsl_types_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/llsw/freertos_osswr" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-asw-2f-components-2f-plantcontrolsystem-2f-src

clean-asw-2f-components-2f-plantcontrolsystem-2f-src:
	-$(RM) ./asw/components/plantcontrolsystem/src/PlantControlSystemB.d ./asw/components/plantcontrolsystem/src/PlantControlSystemB.o ./asw/components/plantcontrolsystem/src/PlantControlSystemS.d ./asw/components/plantcontrolsystem/src/PlantControlSystemS.o

.PHONY: clean-asw-2f-components-2f-plantcontrolsystem-2f-src

