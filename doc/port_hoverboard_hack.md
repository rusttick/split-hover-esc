# Porting Hoverboard-Firmware-Hack-Gen2.x-MM32 to Open Source Toolchains

Research date: January 2025

## Overview

This document details a process for making **minimal additions** to the Hoverboard-Firmware-Hack-Gen2.x-MM32 project to enable dual-toolchain compilation:

1. **Existing**: Keil MDK on Windows (unchanged)
2. **New**: arm-none-eabi-gcc on macOS and Linux

The goal is **additive-only changes** - no modifications to existing files that could break Keil builds.

## Project Analysis

### Current Build System

| Component      | Current (Keil)                  | Notes                |
|----------------|---------------------------------|----------------------|
| Compiler       | ARMCLANG v6.19                  | LLVM-based           |
| Project file   | `Hoverboard.uvprojx`            | XML format           |
| Startup files  | `startup_mm32spin06xx_s_keil.s` | ARM Assembler syntax |
| Linker scripts | Auto-generated scatter files    | Not explicit         |
| HAL Library    | MindMotion proprietary          | C source included    |

### Target MCUs

| MCU          | Core      | RAM               | Flash              | Startup File                  |
|--------------|-----------|-------------------|--------------------|-------------------------------|
| MM32SPIN05PF | Cortex-M0 | 4 KB @ 0x20000000 | 32 KB @ 0x08000000 | startup_mm32spin06xx_s_keil.s |
| MM32SPIN06PF | Cortex-M0 | 4 KB @ 0x20000000 | 32 KB @ 0x08000000 | startup_mm32spin06xx_s_keil.s |
| MM32SPIN25PF | Cortex-M0 | 8 KB @ 0x20000000 | 32 KB @ 0x08000000 | startup_MM32SPIN2xx_p.s       |

System clock: 72 MHz from internal HSI via PLL

### Directory Structure (Current)

```
HoverBoardMindMotion/
├── Inc/                           # Application headers
├── Src/                           # Application source (17 files)
├── Library/
│   ├── MM32SPIN05/HAL_Lib/        # HAL for SPIN05/06
│   └── MM32SPIN25/HAL_lib/        # HAL for SPIN25
├── RTE/Device/                    # Keil device-specific files
│   ├── MM32SPIN05PF/
│   │   ├── startup_mm32spin06xx_s_keil.s  # Keil ARM Assembler
│   │   └── system_mm32spin06xx_s.c        # System init (C)
│   ├── MM32SPIN06PF/
│   └── MM32SPIN25PF/
├── Hoverboard.uvprojx             # Keil project
└── Objects/                       # Build output
```

---

## Porting Strategy: Minimal Additions

### New Files to Add

```
HoverBoardMindMotion/
├── gcc/                           # NEW: GCC-specific files
│   ├── startup_mm32spin05_gcc.s   # GCC startup for SPIN05/06
│   ├── startup_mm32spin25_gcc.s   # GCC startup for SPIN25
│   ├── mm32spin05.ld              # Linker script SPIN05
│   ├── mm32spin06.ld              # Linker script SPIN06
│   ├── mm32spin25.ld              # Linker script SPIN25
│   └── toolchain.mk               # Common compiler flags
├── Makefile                       # NEW: Main build file
└── (existing files unchanged)
```

---

## Step 1: Create GCC Startup Files

The Keil startup file uses ARM Assembler syntax which is incompatible with GNU AS. Create new startup files in `gcc/` directory.

### File: `gcc/startup_mm32spin05_gcc.s`

Convert the Keil syntax to GNU AS syntax:

| Keil ARM Assembler   | GNU AS Equivalent                 |
|----------------------|-----------------------------------|
| `EQU`                | `.equ`                            |
| `AREA name, DATA`    | `.section .name`                  |
| `DCD value`          | `.word value`                     |
| `SPACE n`            | `.space n`                        |
| `PROC` / `ENDP`      | `.type name, %function` / `.size` |
| `EXPORT name [WEAK]` | `.global name` / `.weak name`     |
| `IMPORT name`        | `.extern name`                    |
| `PRESERVE8`          | (handled by compiler)             |
| `THUMB`              | `.thumb`                          |
| `ALIGN`              | `.align 2`                        |
| `IF :DEF:__MICROLIB` | `#ifdef USE_MICROLIB` (or remove) |
| `B .`                | `b .`                             |

```gas
/* gcc/startup_mm32spin05_gcc.s
 * GCC-compatible startup for MM32SPIN05/06
 * Converted from startup_mm32spin06xx_s_keil.s
 */

    .syntax unified
    .cpu cortex-m0
    .fpu softvfp
    .thumb

/* Stack and Heap sizes */
    .equ Stack_Size, 0x400
    .equ Heap_Size,  0x200

/* Stack section */
    .section .stack, "aw", %nobits
    .align 3
    .global __stack_start
    .global __stack_end
__stack_start:
    .space Stack_Size
__stack_end:

/* Heap section */
    .section .heap, "aw", %nobits
    .align 3
    .global __heap_start
    .global __heap_end
__heap_start:
    .space Heap_Size
__heap_end:

/* Vector table */
    .section .isr_vector, "a", %progbits
    .align 2
    .global __Vectors
    .global __Vectors_End
    .global __Vectors_Size

__Vectors:
    .word __stack_end           /* Top of Stack */
    .word Reset_Handler         /* Reset Handler */
    .word NMI_Handler           /* NMI Handler */
    .word HardFault_Handler     /* Hard Fault Handler */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word SVC_Handler           /* SVCall Handler */
    .word 0                     /* Reserved */
    .word 0                     /* Reserved */
    .word PendSV_Handler        /* PendSV Handler */
    .word SysTick_Handler       /* SysTick Handler */

    /* External Interrupts */
    .word WWDG_IRQHandler                /* Window Watchdog */
    .word PVD_IRQHandler                 /* PVD through EXTI */
    .word RTC_BKP_IRQHandler             /* RTC through EXTI */
    .word FLASH_IRQHandler               /* FLASH */
    .word RCC_CRS_IRQHandler             /* RCC & CRS */
    .word EXTI0_1_IRQHandler             /* EXTI Line 0-1 */
    .word EXTI2_3_IRQHandler             /* EXTI Line 2-3 */
    .word EXTI4_15_IRQHandler            /* EXTI Line 4-15 */
    .word HWDIV_IRQHandler               /* HWDIV */
    .word DMA1_Channel1_IRQHandler       /* DMA1 Channel 1 */
    .word DMA1_Channel2_3_IRQHandler     /* DMA1 Channel 2-3 */
    .word DMA1_Channel4_5_IRQHandler     /* DMA1 Channel 4-5 */
    .word ADC1_COMP_IRQHandler           /* ADC1 & COMP */
    .word TIM1_BRK_UP_TRG_COM_IRQHandler /* TIM1 Break/Update/Trigger */
    .word TIM1_CC_IRQHandler             /* TIM1 Capture Compare */
    .word TIM2_IRQHandler                /* TIM2 */
    .word TIM3_IRQHandler                /* TIM3 */
    .word 0                              /* Reserved */
    .word 0                              /* Reserved */
    .word TIM14_IRQHandler               /* TIM14 */
    .word 0                              /* Reserved */
    .word TIM16_IRQHandler               /* TIM16 */
    .word TIM17_IRQHandler               /* TIM17 */
    .word I2C1_IRQHandler                /* I2C1 */
    .word 0                              /* Reserved */
    .word SPI1_IRQHandler                /* SPI1 */
    .word SPI2_IRQHandler                /* SPI2 */
    .word UART1_IRQHandler               /* UART1 */
    .word UART2_IRQHandler               /* UART2 */
    .word 0                              /* Reserved */
    .word CAN_IRQHandler                 /* CAN */
    .word 0                              /* Reserved */

__Vectors_End:
    .equ __Vectors_Size, __Vectors_End - __Vectors

/* Reset Handler */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer */
    ldr r0, =__stack_end
    msr msp, r0

    /* RP enable (MM32 specific) */
    ldr r0, =0x40022060
    ldr r1, =0x00000001
    str r1, [r0]

    /* Check if boot from test memory */
    ldr r0, =0x00000004
    ldr r1, [r0]
    lsrs r1, r1, #24
    ldr r2, =0x1F
    cmp r1, r2
    bne ApplicationStart

    /* SYSCFG clock enable */
    ldr r0, =0x40021018
    ldr r1, =0x00000001
    str r1, [r0]

    /* Set CFGR1 for flash remap */
    ldr r0, =0x40010000
    ldr r1, =0x00000000
    str r1, [r0]

ApplicationStart:
    /* Copy .data from flash to RAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    movs r3, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit

    /* Zero .bss */
    ldr r2, =_sbss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    str r3, [r2]
    adds r2, r2, #4

LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss

    /* Call SystemInit */
    bl SystemInit

    /* Call main */
    bl main

    /* Loop forever if main returns */
    b .

    .size Reset_Handler, .-Reset_Handler

/* Default exception handlers (weak, can be overridden) */
    .section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b .
    .size Default_Handler, .-Default_Handler

/* Macro for weak handlers */
    .macro def_irq_handler handler_name
    .weak \handler_name
    .thumb_set \handler_name, Default_Handler
    .endm

    def_irq_handler NMI_Handler
    def_irq_handler HardFault_Handler
    def_irq_handler SVC_Handler
    def_irq_handler PendSV_Handler
    def_irq_handler SysTick_Handler
    def_irq_handler WWDG_IRQHandler
    def_irq_handler PVD_IRQHandler
    def_irq_handler RTC_BKP_IRQHandler
    def_irq_handler FLASH_IRQHandler
    def_irq_handler RCC_CRS_IRQHandler
    def_irq_handler EXTI0_1_IRQHandler
    def_irq_handler EXTI2_3_IRQHandler
    def_irq_handler EXTI4_15_IRQHandler
    def_irq_handler HWDIV_IRQHandler
    def_irq_handler DMA1_Channel1_IRQHandler
    def_irq_handler DMA1_Channel2_3_IRQHandler
    def_irq_handler DMA1_Channel4_5_IRQHandler
    def_irq_handler ADC1_COMP_IRQHandler
    def_irq_handler TIM1_BRK_UP_TRG_COM_IRQHandler
    def_irq_handler TIM1_CC_IRQHandler
    def_irq_handler TIM2_IRQHandler
    def_irq_handler TIM3_IRQHandler
    def_irq_handler TIM14_IRQHandler
    def_irq_handler TIM16_IRQHandler
    def_irq_handler TIM17_IRQHandler
    def_irq_handler I2C1_IRQHandler
    def_irq_handler SPI1_IRQHandler
    def_irq_handler SPI2_IRQHandler
    def_irq_handler UART1_IRQHandler
    def_irq_handler UART2_IRQHandler
    def_irq_handler CAN_IRQHandler

    .end
```

---

## Step 2: Create Linker Scripts

### File: `gcc/mm32spin05.ld`

```ld
/* Linker script for MM32SPIN05PF
 * RAM:   4 KB @ 0x20000000
 * Flash: 32 KB @ 0x08000000 (8 KB usable on some variants)
 */

ENTRY(Reset_Handler)

_Min_Heap_Size  = 0x200;  /* 512 bytes */
_Min_Stack_Size = 0x400;  /* 1 KB */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS
{
    /* Vector table at start of flash */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Code */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* Read-only data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM exception unwinding */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    /* Used by startup to initialize .data */
    _sidata = LOADADDR(.data);

    /* Initialized data (copied from flash to RAM) */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    /* Uninitialized data (zeroed) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* Heap grows up from end of BSS */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /* Discard debug from standard libraries */
    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* Provide symbols for startup code */
PROVIDE(_stack = ORIGIN(RAM) + LENGTH(RAM));
```

### File: `gcc/mm32spin06.ld`

Same as mm32spin05.ld (identical memory map).

### File: `gcc/mm32spin25.ld`

```ld
/* Linker script for MM32SPIN25PF
 * RAM:   8 KB @ 0x20000000
 * Flash: 32 KB @ 0x08000000
 */

ENTRY(Reset_Handler)

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* ... (same SECTIONS as mm32spin05.ld) ... */
```

---

## Step 3: Create the Makefile

### File: `Makefile`

```makefile
# Makefile for Hoverboard-Firmware-Hack-Gen2.x-MM32
# Supports: MM32SPIN05, MM32SPIN06, MM32SPIN25
# Usage: make TARGET=MM32SPIN05

#------------------------------------------------------------------------------
# Configuration
#------------------------------------------------------------------------------

# Default target
TARGET ?= MM32SPIN06

# Toolchain
PREFIX ?= arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
CP      = $(PREFIX)objcopy
SZ      = $(PREFIX)size
HEX     = $(CP) -O ihex
BIN     = $(CP) -O binary -S

# Build directory
BUILD_DIR = build/$(TARGET)

#------------------------------------------------------------------------------
# Target-specific configuration
#------------------------------------------------------------------------------

ifeq ($(TARGET),MM32SPIN05)
    CPU        = -mcpu=cortex-m0
    HAL_DIR    = Library/MM32SPIN05/HAL_Lib
    STARTUP    = gcc/startup_mm32spin05_gcc.s
    LDSCRIPT   = gcc/mm32spin05.ld
    DEVICE_DIR = RTE/Device/MM32SPIN05PF
    C_DEFS     = -DMM32SPIN05
else ifeq ($(TARGET),MM32SPIN06)
    CPU        = -mcpu=cortex-m0
    HAL_DIR    = Library/MM32SPIN05/HAL_Lib
    STARTUP    = gcc/startup_mm32spin05_gcc.s
    LDSCRIPT   = gcc/mm32spin06.ld
    DEVICE_DIR = RTE/Device/MM32SPIN06PF
    C_DEFS     = -DMM32SPIN06
else ifeq ($(TARGET),MM32SPIN25)
    CPU        = -mcpu=cortex-m0
    HAL_DIR    = Library/MM32SPIN25/HAL_lib
    STARTUP    = gcc/startup_mm32spin25_gcc.s
    LDSCRIPT   = gcc/mm32spin25.ld
    DEVICE_DIR = RTE/Device/MM32SPIN25PF
    C_DEFS     = -DMM32SPIN25 -DTARGET_MM32SPIN25
else
    $(error Unknown TARGET: $(TARGET). Use MM32SPIN05, MM32SPIN06, or MM32SPIN25)
endif

#------------------------------------------------------------------------------
# Source files
#------------------------------------------------------------------------------

# Application sources
C_SOURCES = \
    Src/main.c \
    Src/bldc.c \
    Src/calculation.c \
    Src/clarke.c \
    Src/delay.c \
    Src/FOC_Math.c \
    Src/hallhandle.c \
    Src/hardware.c \
    Src/initialize.c \
    Src/interrupt.c \
    Src/ipark.c \
    Src/park.c \
    Src/PID.c \
    Src/pwm_gen.c \
    Src/remoteUartBus.c \
    Src/sim_eeprom.c \
    Src/uart.c \
    $(DEVICE_DIR)/system_mm32spin06xx_s.c

# HAL sources
C_SOURCES += $(wildcard $(HAL_DIR)/Src/*.c)

# Assembly sources
ASM_SOURCES = $(STARTUP)

#------------------------------------------------------------------------------
# Include paths
#------------------------------------------------------------------------------

C_INCLUDES = \
    -IInc \
    -I$(HAL_DIR)/Inc \
    -I$(DEVICE_DIR) \
    -IRTE/_$(TARGET)

#------------------------------------------------------------------------------
# Compiler flags
#------------------------------------------------------------------------------

# CPU flags
MCU = $(CPU) -mthumb -mfloat-abi=soft

# Optimization (Os for size, O2 for speed)
OPT = -Os

# C flags
CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT)
CFLAGS += -Wall -fdata-sections -ffunction-sections
CFLAGS += -std=c99
CFLAGS += -g -gdwarf-2

# Assembly flags
ASFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT)
ASFLAGS += -Wall -fdata-sections -ffunction-sections

# Linker flags
LDFLAGS  = $(MCU) -specs=nano.specs -specs=nosys.specs
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -lm

#------------------------------------------------------------------------------
# Build rules
#------------------------------------------------------------------------------

# Output files
OUTPUT = $(BUILD_DIR)/$(TARGET)

# Object files
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Default target
all: $(OUTPUT).elf $(OUTPUT).hex $(OUTPUT).bin
	$(SZ) $(OUTPUT).elf

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(OUTPUT).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(OUTPUT).hex: $(OUTPUT).elf
	$(HEX) $< $@

$(OUTPUT).bin: $(OUTPUT).elf
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#------------------------------------------------------------------------------
# Utility targets
#------------------------------------------------------------------------------

clean:
	rm -rf build

# Flash using Black Magic Probe
flash-bmp: $(OUTPUT).elf
	arm-none-eabi-gdb -nx --batch \
		-ex 'target extended-remote /dev/cu.usbmodem*1' \
		-ex 'monitor swdp_scan' \
		-ex 'attach 1' \
		-ex 'load' \
		-ex 'compare-sections' \
		-ex 'kill' \
		$(OUTPUT).elf

# Flash using OpenOCD (requires MM32 support)
flash-openocd: $(OUTPUT).elf
	openocd -f interface/stlink.cfg -f target/mm32.cfg \
		-c "program $(OUTPUT).elf verify reset exit"

# Print size info
size: $(OUTPUT).elf
	$(SZ) --format=berkeley $(OUTPUT).elf

.PHONY: all clean flash-bmp flash-openocd size
```

---

## Step 4: Handle System Initialization Compatibility

The existing `system_mm32spin06xx_s.c` should work with GCC. However, check for these potential issues:

### Data Type Compatibility

The HAL uses custom types `u8`, `u16`, `u32`, `s8`, `s16`, `s32`, `__I`, `__O`, `__IO`. These should be defined in `dtype.h` or `mm32_device.h`. Verify they resolve correctly:

```c
typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;
typedef signed char        s8;
typedef signed short       s16;
typedef signed int         s32;

#define __I  volatile const  /* read-only */
#define __O  volatile        /* write-only */
#define __IO volatile        /* read-write */
```

### Potential Keil Intrinsics

If the code uses Keil-specific intrinsics, create a compatibility header:

#### File: `gcc/gcc_compat.h` (if needed)

```c
#ifndef GCC_COMPAT_H
#define GCC_COMPAT_H

#ifdef __GNUC__

/* Keil -> GCC intrinsic mappings */
#define __nop()     __asm__ volatile ("nop")
#define __wfi()     __asm__ volatile ("wfi")
#define __wfe()     __asm__ volatile ("wfe")
#define __sev()     __asm__ volatile ("sev")

/* Bit manipulation (Cortex-M0 doesn't have CLZ, etc.) */
static inline unsigned int __clz(unsigned int x) {
    unsigned int n = 0;
    if (x == 0) return 32;
    while (!(x & 0x80000000)) { n++; x <<= 1; }
    return n;
}

#endif /* __GNUC__ */
#endif /* GCC_COMPAT_H */
```

---

## Step 5: RTE_Components.h

Create a GCC-compatible version if needed:

#### File: `RTE/_MM32SPIN05_GCC/RTE_Components.h`

```c
#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

#define CMSIS_device_header "mm32_device.h"

#endif /* RTE_COMPONENTS_H */
```

---

## Step 6: Installation & Build

### Install ARM GCC Toolchain

**macOS (Apple Silicon)**:
```bash
# Official ARM build
wget https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.pkg
sudo installer -pkg arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.pkg -target /

# Add to PATH
export PATH="/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin:$PATH"
```

**macOS (Homebrew)**:
```bash
brew install --cask gcc-arm-embedded
```

**Linux (Ubuntu/Debian)**:
```bash
sudo apt install gcc-arm-none-eabi
```

### Build

```bash
cd HoverBoardMindMotion

# Build for MM32SPIN06 (default)
make

# Build for specific target
make TARGET=MM32SPIN05
make TARGET=MM32SPIN25

# Clean and rebuild
make clean
make TARGET=MM32SPIN06
```

### Expected Output

```
arm-none-eabi-size build/MM32SPIN06/MM32SPIN06.elf
   text    data     bss     dec     hex filename
  12456     120     892   13468    349c build/MM32SPIN06/MM32SPIN06.elf
```

---

## Step 7: Flashing

### Black Magic Probe (Recommended)

Black Magic Debug has [native MM32 support](https://black-magic.org/hacking/target-mm32.html).

```bash
make flash-bmp
```

### ST-Link with OpenOCD

OpenOCD does NOT have native MM32 support. MM32 flash write timing differs from STM32. Options:

1. Use a [patched OpenOCD with MM32 support](https://github.com/nickvyg/openocd-mm32)
2. Use Black Magic Probe
3. Use MindMotion's MM32-DAP

---

## Summary of Files to Add

| File                           | Purpose                             |
|--------------------------------|-------------------------------------|
| `gcc/startup_mm32spin05_gcc.s` | GCC startup for SPIN05/06           |
| `gcc/startup_mm32spin25_gcc.s` | GCC startup for SPIN25              |
| `gcc/mm32spin05.ld`            | Linker script for SPIN05            |
| `gcc/mm32spin06.ld`            | Linker script for SPIN06            |
| `gcc/mm32spin25.ld`            | Linker script for SPIN25            |
| `gcc/gcc_compat.h`             | Intrinsic compatibility (if needed) |
| `Makefile`                     | Build system                        |

**Total: 6-7 new files, 0 existing files modified**

---

## Verification Checklist

- [ ] Toolchain installed: `arm-none-eabi-gcc --version`
- [ ] Build succeeds: `make TARGET=MM32SPIN06`
- [ ] Binary size reasonable (< 32KB for flash)
- [ ] .hex file generated
- [ ] Keil build still works (test on Windows)
- [ ] Flash and run on hardware

---

## Troubleshooting

### "undefined reference to `__aeabi_*`"

Missing libgcc. Add `-lgcc` to LDFLAGS or ensure `specs=nano.specs` is used.

### "region FLASH overflowed"

Binary too large. Try:
- Change optimization to `-Os`
- Remove unused HAL modules from build
- Check for debug symbols: add `-g0` to remove

### "SystemInit not found"

The `system_mm32spin06xx_s.c` isn't being compiled. Check that `DEVICE_DIR` in Makefile points to correct variant.

### Different behavior than Keil build

- Verify `SystemCoreClock` is set to 72000000
- Check compiler optimization levels match
- Ensure `.data` and `.bss` initialization in startup is correct

---

## References

- [iclite/mm32_startup](https://github.com/iclite/mm32_startup) - GCC-compatible startup files for MM32
- [Black Magic Debug MM32 Support](https://black-magic.org/hacking/target-mm32.html)
- [ARM GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
- [Porting from Keil to GCC](https://www.renesas.com/en/document/apn/b-024-porting-keil-uvision-project-gnu-chain-tool)
