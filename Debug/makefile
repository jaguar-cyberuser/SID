################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := SID_IDE_TIM4_DMA_IT
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SID_IDE_TIM4_DMA_IT.elf \

MAP_FILES += \
SID_IDE_TIM4_DMA_IT.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SID_IDE_TIM4_DMA_IT.list \

OBJCOPY_HEX += \
SID_IDE_TIM4_DMA_IT.hex \

OBJCOPY_BIN += \
SID_IDE_TIM4_DMA_IT.bin \


# All Target
all: main-build

# Main-build Target
main-build: SID_IDE_TIM4_DMA_IT.elf secondary-outputs

# Tool invocations
SID_IDE_TIM4_DMA_IT.elf SID_IDE_TIM4_DMA_IT.map: $(OBJS) $(USER_OBJS) D:\WORK\BNGF\SID\Projects\SID_IDE_TIM4_DMA_IT\STM32F103CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "SID_IDE_TIM4_DMA_IT.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\WORK\BNGF\SID\Projects\SID_IDE_TIM4_DMA_IT\STM32F103CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SID_IDE_TIM4_DMA_IT.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SID_IDE_TIM4_DMA_IT.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SID_IDE_TIM4_DMA_IT.list"
	@echo 'Finished building: $@'
	@echo ' '

SID_IDE_TIM4_DMA_IT.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "SID_IDE_TIM4_DMA_IT.hex"
	@echo 'Finished building: $@'
	@echo ' '

SID_IDE_TIM4_DMA_IT.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SID_IDE_TIM4_DMA_IT.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) SID_IDE_TIM4_DMA_IT.bin SID_IDE_TIM4_DMA_IT.elf SID_IDE_TIM4_DMA_IT.hex SID_IDE_TIM4_DMA_IT.list SID_IDE_TIM4_DMA_IT.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
