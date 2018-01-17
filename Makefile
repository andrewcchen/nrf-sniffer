TARGET ?= nrf51

INCLUDE_DIRS := -Iinclude

ifeq ($(TARGET), nrf51)
ARCH_FLAGS := -mthumb -mcpu=cortex-m0 -mfloat-abi=soft
LD_LIBS := -Lsrc/nrf51 -Tnrf51.ld
DEFS := -DNRF51=1
OBJS := src/nrf51/system_nrf51.o src/nrf51/gcc_startup_nrf51.S.o
endif

ifeq ($(TARGET), nrf52)
ARCH_FLAGS := -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
LD_LIBS := -Lsrc/nrf52 -Tnrf52.ld
DEFS := -DNRF52=1
OBJS := src/nrf52/system_nrf52.o src/nrf52/gcc_startup_nrf52.S.o
endif

PREFIX ?= arm-none-eabi

CC := $(PREFIX)-gcc
CXX := $(PREFIX)-g++
LD := $(PREFIX)-gcc
AR := $(PREFIX)-ar
AS := $(PREFIX)-as
OBJCOPY := $(PREFIX)-objcopy
OBJDUMP := $(PREFIX)-objdump

CFLAGS ?= -g -O
CFLAGS := -Wall -Wextra -Wmissing-prototypes -Wstrict-prototypes -fno-common $(ARCH_FLAGS) $(CFLAGS)
CPPFLAGS := -MD $(INCLUDE_DIRS) $(DEFS) $(CPPFLAGS)
LDFLAGS := --static $(LD_LIBS) $(ARCH_FLAGS) $(LDFLAGS)


# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

%.S.o: %.S
	@printf "  ASM     $(*).S\n"
	$(Q)$(CC) -x assembler-with-cpp -o $(*).S.o -c $(*).S

%.o: %.c
	@printf "  CC      $(*).c\n"
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).c

%.elf: src/%.o $(OBJS)
	@printf "  LD      $(*).elf\n"
	$(Q)$(LD) $(LDFLAGS) $(^) -o $(*).elf

all: advmon.elf

clean:
	@printf "  CLEAN\n"
	$(Q)find -name \*.o -delete
	$(Q)find -name \*.d -delete
	$(Q)find -name \*.elf -delete

.PHONY: all clean
.SECONDARY:
