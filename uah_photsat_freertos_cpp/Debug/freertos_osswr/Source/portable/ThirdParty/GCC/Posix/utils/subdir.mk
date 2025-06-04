################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c 

C_DEPS += \
./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/wait_for_event.d 

OBJS += \
./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/wait_for_event.o 


# Each subdirectory must supply rules for building sources it contributes
freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/%.o: ../freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/%.c freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/config/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/edroombp_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos_osswr-2f-Source-2f-portable-2f-ThirdParty-2f-GCC-2f-Posix-2f-utils

clean-freertos_osswr-2f-Source-2f-portable-2f-ThirdParty-2f-GCC-2f-Posix-2f-utils:
	-$(RM) ./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/wait_for_event.d ./freertos_osswr/Source/portable/ThirdParty/GCC/Posix/utils/wait_for_event.o

.PHONY: clean-freertos_osswr-2f-Source-2f-portable-2f-ThirdParty-2f-GCC-2f-Posix-2f-utils

