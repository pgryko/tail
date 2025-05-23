# Copyright (C) 2019 Luke Barone-Adesi <luke.barone-adesi@unipart.io>
# License: whatever is being used for Tail

# Background:
# Automatically track dependencies without a separate depend target:
# http://make.mad-scientist.net/papers/advanced-auto-dependency-generation/

# Recursive make considered harmful:
# http://lcgapp.cern.ch/project/architecture/recursive_make.pdf
# Modernised example:
# https://www.oreilly.com/library/view/managing-projects-with/0596006101/ch06.html

all: Tail.axf bootloader.axf combined
combined: Tail-combined.hex Tail-combined.bin

CC = arm-none-eabi-gcc
TARGET_ARCH = -mcpu=cortex-m0plus -mthumb
DEBUG_FLAGS += -g -gdwarf-2
CFLAGS += $(DEBUG_FLAGS)
CFLAGS += $(TARGET_ARCH)
CFLAGS += -std=c99 -DNDEBUG=1 -DEFM32HG222F64=1
CFLAGS += -Os -Wall -Wno-main -Wno-unknown-pragmas -fmessage-length=0 -ffunction-sections -fdata-sections
ASFLAGS := -c -x assembler-with-cpp -D__STARTUP_CLEAR_BSS

VERSION=$(shell git describe --tags --always --abbrev=1 --match "v*")

BOOTLOADER_DEFINES := -DBOOTLOADER_VERSION_STRING=\"$(VERSION)\" -DNDEBUG -D_EFM32_HAPPY_FAMILY=1
CFLAGS += $(BOOTLOADER_DEFINES)

SYSLIBS = -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group # TODO rename?

INCDIR_BASE = $(CURDIR)/efm32-base
INCDIRS = $(INCDIR_BASE)/emlib/inc
INCDIRS += $(INCDIR_BASE)/device/EFM32HG/Include
INCDIRS += $(INCDIR_BASE)/cmsis/Include
CFLAGS += $(patsubst %,-I%,$(INCDIRS))

CFLAGS += -DTAIL_VERSION=\"$(VERSION)\"

MODULES := emlib src CMSIS/EFM32HG bootloader_src
SRCS :=
include $(patsubst %,%/module.mk,$(MODULES))

BUILDDIR := .build
$(shell mkdir -p $(BUILDDIR) >/dev/null)
TAIL_OBJECTS := $(patsubst %.c,$(BUILDDIR)/%.o,$(SRCS))

DEPDIR := $(BUILDDIR)/.d
$(shell mkdir -p $(DEPDIR) >/dev/null)

SUBDEPDIRS := $(foreach MB,$(MODULES),$(shell find $(MB) -type d))
$(foreach sdd,$(SUBDEPDIRS),$(shell mkdir -p $(BUILDDIR)/$(sdd) >/dev/null))
$(foreach sdd,$(SUBDEPDIRS),$(shell mkdir -p $(DEPDIR)/$(sdd) >/dev/null))
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) -c
POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

# FIXME: layer cmsis builds more cleanly
CMSIS_EFM32HG_ASM_FLAGS := -c -x assembler-with-cpp
$(BUILDDIR)/$(CMSIS_EFM32HG_SRC_DIRNAME)/%.o: $(CMSIS_EFM32HG_SRC_DIRNAME)/%.s
	$(CC) $(TARGET_ARCH) $(CMSIS_EFM32HG_ASM_FLAGS) $(OUTPUT_OPTION) $<

$(BUILDDIR)/$(BOOTLOADER_SRC_DIRNAME)/%.o: $(BOOTLOADER_SRC_DIRNAME)/%.s
	$(CC) $(TARGET_ARCH) $(ASFLAGS) $(OUTPUT_OPTION) $<

CMSIS_EFM32HG_CFLAGS += -O0 -Wall -fmessage-length=0 -mno-sched-prolog \
    -fno-builtin -ffunction-sections -fdata-sections
$(BUILDDIR)/$(CMSIS_EFM32HG_SRC_DIRNAME)/%.o: $(CMSIS_EFM32HG_SRC_DIRNAME)/%.c
	$(CC) $(DEPFLAGS) $(CFLAGS) $(CMSIS_EFM32HG_CFLAGS) $(OUTPUT_OPTION) -c $<
	$(POSTCOMPILE)

#$(BUILDDIR)/$(BOOTLOADER_SRC_DIRNAME)/startup_efm32.o: $(BOOTLOADER_SRC_DIRNAME)/startup_efm32.s
#	$(CC) $(TARGET_ARCH) $(CMSIS_EFM32HG_ASM_FLAGS) $(OUTPUT_OPTION) $<

%.o : $.c
$(BUILDDIR)/%.o : %.c $(DEPDIR)/%.d
	$(COMPILE.c) $(OUTPUT_OPTION) $<
	$(POSTCOMPILE)

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d

CMSIS_OBJECTS = $(BUILDDIR)/CMSIS/EFM32HG/startup_gcc_efm32hg.o \
                $(BUILDDIR)/CMSIS/EFM32HG/system_efm32hg.o

# Build of tail to be loaded from a bootloader
Tail.axf: LDFLAGS := -T "Tail.ld" -Xlinker --gc-sections -Xlinker -Map="Tail.map" --specs=nano.specs
Tail.axf: $(TAIL_OBJECTS) $(CMSIS_OBJECTS)
	$(CC) $(DEBUG_FLAGS) $(TARGET_ARCH) $(LDFLAGS) -o Tail.axf $(CMSIS_OBJECTS) $(TAIL_OBJECTS) $(SYSLIBS)

	arm-none-eabi-objcopy -O ihex "Tail.axf" "Tail.hex"
	arm-none-eabi-objcopy -O binary "Tail.axf" "Tail.bin"
	arm-none-eabi-objcopy -O srec "Tail.axf" "Tail.s37"
	arm-none-eabi-size "Tail.axf"

BOOTLOADER_OBJECTS := $(patsubst %.c,$(BUILDDIR)/%.o,$(BOOTLOADER_SRCS))
BOOTLOADER_OBJECTS += $(BUILDDIR)/$(BOOTLOADER_SRC_DIRNAME)/startup_gcc_efm32hg.o
bootloader.axf: LDFLAGS := -T "bootloader.ld" -Xlinker --gc-sections -Xlinker -Map="bootloader.map" --specs=nano.specs -nostartfiles
bootloader.axf: $(BOOTLOADER_OBJECTS)
	$(CC) $(DEBUG_FLAGS) $(TARGET_ARCH) $(LDFLAGS) -o bootloader.axf $(BOOTLOADER_OBJECTS) $(SYSLIBS)

	arm-none-eabi-objcopy -O ihex "bootloader.axf" "bootloader.hex"
	arm-none-eabi-objcopy -O binary "bootloader.axf" "bootloader.bin"
	arm-none-eabi-objcopy -O srec "bootloader.axf" "bootloader.s37"
	arm-none-eabi-size "bootloader.axf"

Tail-combined.hex: Tail.axf bootloader.axf
	srec_cat $(foreach file,$^,$(basename $(file)).hex -Intel) -o $@ -Intel

Tail-combined.bin: Tail-combined.hex
	srec_cat $< -Intel -fill 0x0ff -over $< -Intel -o $@ -binary

# TODO: make this more defensive
clean:
	rm -rf $(DEPDIR) $(BUILDDIR)
	rm -f Tail.axf Tail.hex Tail.bin Tail.s37 Tail.map
	rm -f bootloader.axf bootloader.hex bootloader.bin bootloader.s37 bootloader.map
	rm -f Tail-combined.hex Tail-combined.bin

.PHONY: all clean combined

include $(wildcard $(patsubst $(BUILDDIR)/%,$(DEPDIR)/%.d,$(basename $(TAIL_OBJECTS))))
include $(wildcard $(patsubst $(BUILDDIR)/%,$(DEPDIR)/%.d,$(basename $(BOOTLOADER_OBJECTS))))
