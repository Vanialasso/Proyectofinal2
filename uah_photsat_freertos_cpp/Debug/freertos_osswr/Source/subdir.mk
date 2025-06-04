################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../freertos_osswr/Source/croutine.c \
../freertos_osswr/Source/event_groups.c \
../freertos_osswr/Source/list.c \
../freertos_osswr/Source/queue.c \
../freertos_osswr/Source/stream_buffer.c \
../freertos_osswr/Source/tasks.c \
../freertos_osswr/Source/timers.c 

C_DEPS += \
./freertos_osswr/Source/croutine.d \
./freertos_osswr/Source/event_groups.d \
./freertos_osswr/Source/list.d \
./freertos_osswr/Source/queue.d \
./freertos_osswr/Source/stream_buffer.d \
./freertos_osswr/Source/tasks.d \
./freertos_osswr/Source/timers.d 

OBJS += \
./freertos_osswr/Source/croutine.o \
./freertos_osswr/Source/event_groups.o \
./freertos_osswr/Source/list.o \
./freertos_osswr/Source/queue.o \
./freertos_osswr/Source/stream_buffer.o \
./freertos_osswr/Source/tasks.o \
./freertos_osswr/Source/timers.o 


# Each subdirectory must supply rules for building sources it contributes
freertos_osswr/Source/%.o: ../freertos_osswr/Source/%.c freertos_osswr/Source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/config/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/edroombp_swr/include" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr" -I"/home/opolo70/OBDH-workspace/edroom_freertos_plantcontrol_students/freertos_osswr/Source/portable/ThirdParty/GCC/Posix" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-freertos_osswr-2f-Source

clean-freertos_osswr-2f-Source:
	-$(RM) ./freertos_osswr/Source/croutine.d ./freertos_osswr/Source/croutine.o ./freertos_osswr/Source/event_groups.d ./freertos_osswr/Source/event_groups.o ./freertos_osswr/Source/list.d ./freertos_osswr/Source/list.o ./freertos_osswr/Source/queue.d ./freertos_osswr/Source/queue.o ./freertos_osswr/Source/stream_buffer.d ./freertos_osswr/Source/stream_buffer.o ./freertos_osswr/Source/tasks.d ./freertos_osswr/Source/tasks.o ./freertos_osswr/Source/timers.d ./freertos_osswr/Source/timers.o

.PHONY: clean-freertos_osswr-2f-Source

