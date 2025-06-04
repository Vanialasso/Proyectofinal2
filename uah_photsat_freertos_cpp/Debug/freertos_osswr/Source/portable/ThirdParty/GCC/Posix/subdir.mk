################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos_osswr/Source/portable/ThirdParty/GCC/Posix/port.c 

C_DEPS += \
./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/port.d 

OBJS += \
./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/port.o 


# Each subdirectory must supply rules for building sources it contributes
freertos_osswr/Source/portable/ThirdParty/GCC/Posix/%.o: ../freertos_osswr/Source/portable/ThirdParty/GCC/Posix/%.c freertos_osswr/Source/portable/ThirdParty/GCC/Posix/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/config/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/edroombp_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos_osswr-2f-Source-2f-portable-2f-ThirdParty-2f-GCC-2f-Posix

clean-freertos_osswr-2f-Source-2f-portable-2f-ThirdParty-2f-GCC-2f-Posix:
	-$(RM) ./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/port.d ./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/port.o

.PHONY: clean-freertos_osswr-2f-Source-2f-portable-2f-ThirdParty-2f-GCC-2f-Posix

