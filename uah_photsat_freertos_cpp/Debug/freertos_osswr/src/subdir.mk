################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos_osswr/src/croutine.c \
../freertos_osswr/src/event_groups.c \
../freertos_osswr/src/list.c \
../freertos_osswr/src/queue.c \
../freertos_osswr/src/tasks.c \
../freertos_osswr/src/timers.c 

OBJS += \
./freertos_osswr/src/croutine.o \
./freertos_osswr/src/event_groups.o \
./freertos_osswr/src/list.o \
./freertos_osswr/src/queue.o \
./freertos_osswr/src/tasks.o \
./freertos_osswr/src/timers.o 

C_DEPS += \
./freertos_osswr/src/croutine.d \
./freertos_osswr/src/event_groups.d \
./freertos_osswr/src/list.d \
./freertos_osswr/src/queue.d \
./freertos_osswr/src/tasks.d \
./freertos_osswr/src/timers.d 


# Each subdirectory must supply rules for building sources it contributes
freertos_osswr/src/%.o: ../freertos_osswr/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/freertos_osswr/include" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/config/include" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/edroombp_swr/include" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/freertos_osswr/src/portable/GCC/Posix" -I"/home/osboxes/SEAR_workspace/edroom_freertos_plantcontrol/freertos_osswr/include/public" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


