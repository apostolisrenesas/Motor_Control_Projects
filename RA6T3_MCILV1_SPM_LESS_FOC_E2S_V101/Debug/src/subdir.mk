################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/hal_entry.c 

C_DEPS += \
./src/hal_entry.d 

OBJS += \
./src/hal_entry.o 

SREC += \
RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101.srec 

MAP += \
RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101.map 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -g -gdwarf-4 -D_RA_CORE=CM33 -D_RENESAS_RA_ -D_RA_ORDINAL=1 -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/src" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra/fsp/inc" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra/fsp/inc/api" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra/fsp/inc/instances" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra_gen" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra_cfg/fsp_cfg/bsp" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra_cfg/fsp_cfg" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/src/application/main" -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/src/application/user_interface/ics" -I"." -I"C:/Users/a5143778/e2_studio/customized_workspace/RA6T3_MCILV1_SPM_LESS_FOC_E2S_V101/ra/arm/CMSIS_6/CMSIS/Core/Include" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" -x c "$<")
	@echo Building file: $< && arm-none-eabi-gcc @"$@.in"

