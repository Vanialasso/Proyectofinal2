################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos_osswr/src/portable/MemMang/heap_3.c 

OBJS += \
./freertos_osswr/src/portable/MemMang/heap_3.o 

C_DEPS += \
./freertos_osswr/src/portable/MemMang/heap_3.d 


# Each subdirectory must supply rules for building sources it contributes
freertos_osswr/src/portable/MemMang/%.o: ../freertos_osswr/src/portable/MemMang/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/freertos_osswr/include" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/config/include" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/edroombp_swr/include" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/freertos_osswr/src/portable/GCC/Posix" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/freertos_osswr/include/public" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


