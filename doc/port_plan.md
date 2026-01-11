# Implementation Plan: Porting Hoverboard Firmware to GCC

This plan divides the porting work into small, independently testable components. Each phase verifies specific assumptions before proceeding.

## Overview

```
Phase 1: Toolchain Setup          [Independent]
Phase 2: Minimal Assembly Test    [Depends on 1]
Phase 3: Linker Script Test       [Depends on 2]
Phase 4: C Compilation Test       [Depends on 1]
Phase 5: HAL Library Test         [Depends on 4]
Phase 6: System Init Test         [Depends on 5]
Phase 7: Integration Build        [Depends on 3, 6]
Phase 8: Binary Verification      [Depends on 7]
Phase 9: Hardware Validation      [Depends on 8]
```

---

## Directory Structure

All GCC-related files including tests live under `gcc/`:

```
HoverBoardMindMotion/
├── gcc/
│   ├── Makefile                    # Main build + test targets
│   ├── toolchain.mk                # Compiler flags and paths
│   │
│   ├── startup_mm32spin05_gcc.s    # Production startup (SPIN05/06)
│   ├── startup_mm32spin25_gcc.s    # Production startup (SPIN25)
│   ├── mm32spin05.ld               # Production linker script
│   ├── mm32spin06.ld
│   ├── mm32spin25.ld
│   │
│   └── test/                       # Test files
│       ├── run_tests.sh            # Test runner script
│       ├── test_results/           # Build artifacts (gitignored)
│       │
│       ├── phase1_toolchain/       # Phase 1: Toolchain verification
│       │   └── test_cortexm0.c
│       │
│       ├── phase2_assembly/        # Phase 2: Assembly syntax
│       │   ├── test_startup_minimal.s
│       │   └── test_override.c
│       │
│       ├── phase3_linker/          # Phase 3: Linker script
│       │   └── test_minimal.ld
│       │
│       ├── phase4_c_compile/       # Phase 4: C compilation
│       │   └── test_types.c
│       │
│       ├── phase5_hal/             # Phase 5: HAL library
│       │   └── test_hal_link.c
│       │
│       └── phase6_system/          # Phase 6: System init
│           └── (uses production files)
```

## Make Targets

Standard targets following GNU conventions:

| Target | Description |
|--------|-------------|
| `make` | Build firmware (default target) |
| `make all` | Same as `make` |
| `make clean` | Remove build artifacts |
| `make check` | Run all tests (phases 1-8) |
| `make check-env` | Phase 1: Verify toolchain installed |
| `make check-asm` | Phase 2: Test assembly syntax |
| `make check-ld` | Phase 3: Test linker scripts |
| `make check-compile` | Phase 4: Test C compilation |
| `make check-hal` | Phase 5: Test HAL library |
| `make check-system` | Phase 6: Test system init |
| `make check-build` | Phase 7: Full integration build |
| `make check-binary` | Phase 8: Verify binary output |
| `make flash` | Flash to hardware (Phase 9) |
| `make test` | Alias for `make check` |

Usage:
```bash
# Run all verification tests
make check

# Run just environment check
make check-env

# Build after tests pass
make TARGET=MM32SPIN06
```

---

## Phase 1: Toolchain Setup

**Goal**: Verify arm-none-eabi-gcc is installed and can target Cortex-M0.

**Test directory**: `gcc/test/phase1_toolchain/`

### Step 1.1: Check toolchain installation

```bash
arm-none-eabi-gcc --version
arm-none-eabi-as --version
arm-none-eabi-ld --version
arm-none-eabi-objcopy --version
arm-none-eabi-size --version
```

**Pass criteria**: All commands return version info without errors.

### Step 1.2: Test Cortex-M0 compilation

Create `gcc/test/phase1_toolchain/test_cortexm0.c`:
```c
int main(void) {
    volatile int x = 42;
    return x;
}
```

Compile:
```bash
cd gcc/test/phase1_toolchain
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -c test_cortexm0.c -o ../test_results/test_cortexm0.o
arm-none-eabi-objdump -d ../test_results/test_cortexm0.o
```

**Pass criteria**:
- Compilation succeeds
- Objdump shows Thumb instructions (16-bit opcodes)
- No floating-point instructions present

### Step 1.3: Test nano specs

```bash
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb --specs=nano.specs --specs=nosys.specs -c test_cortexm0.c -o ../test_results/test_cortexm0.o
```

**Pass criteria**: Compiles without "cannot find specs file" error.

---

## Phase 2: Minimal Assembly Test

**Goal**: Verify GCC assembler can process our startup file syntax.

**Test directory**: `gcc/test/phase2_assembly/`

### Step 2.1: Create minimal startup test

Create `gcc/test/phase2_assembly/test_startup_minimal.s`:
```gas
    .syntax unified
    .cpu cortex-m0
    .fpu softvfp
    .thumb

    .equ Stack_Size, 0x400

    .section .stack, "aw", %nobits
    .align 3
__stack_start:
    .space Stack_Size
__stack_end:

    .section .isr_vector, "a", %progbits
    .align 2
    .global __Vectors
__Vectors:
    .word __stack_end
    .word Reset_Handler
    .word Default_Handler
    .word Default_Handler

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr r0, =__stack_end
    msr msp, r0
    b .
    .size Reset_Handler, .-Reset_Handler

    .section .text.Default_Handler
    .weak Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b .
    .size Default_Handler, .-Default_Handler

    .end
```

Assemble:
```bash
cd gcc/test/phase2_assembly
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -c test_startup_minimal.s -o ../test_results/test_startup.o
arm-none-eabi-objdump -h ../test_results/test_startup.o
arm-none-eabi-objdump -t ../test_results/test_startup.o
```

**Pass criteria**:
- Assembly succeeds without errors
- Sections `.isr_vector`, `.stack`, `.text.Reset_Handler` are present
- Symbols `__Vectors`, `Reset_Handler`, `__stack_end` are defined

### Step 2.2: Test weak symbol behavior

Create `gcc/test/phase2_assembly/test_override.c`:
```c
void Reset_Handler(void) {
    while(1);
}
```

```bash
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -c test_override.c -o ../test_results/test_override.o
arm-none-eabi-nm ../test_results/test_startup.o | grep Reset_Handler
arm-none-eabi-nm ../test_results/test_override.o | grep Reset_Handler
```

**Pass criteria**:
- `test_startup.o` shows `W` (weak) for Reset_Handler
- `test_override.o` shows `T` (text/strong) for Reset_Handler

---

## Phase 3: Linker Script Test

**Goal**: Verify linker script produces correct memory layout.

**Test directory**: `gcc/test/phase3_linker/`

### Step 3.1: Create minimal linker script

Create `gcc/test/phase3_linker/test_minimal.ld`:
```ld
ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    .stack :
    {
        . = ALIGN(8);
        *(.stack)
    } >RAM
}
```

### Step 3.2: Link minimal test

```bash
cd gcc/test/phase3_linker
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -nostartfiles \
    -T test_minimal.ld ../test_results/test_startup.o -o ../test_results/test_minimal.elf
arm-none-eabi-objdump -h ../test_results/test_minimal.elf
arm-none-eabi-size ../test_results/test_minimal.elf
```

**Pass criteria**:
- Linking succeeds
- `.isr_vector` is at 0x08000000
- `.text` follows `.isr_vector` in FLASH
- `.data` and `.bss` are in RAM region (0x20000000+)

### Step 3.3: Verify vector table contents

```bash
arm-none-eabi-objdump -s -j .isr_vector ../test_results/test_minimal.elf
```

**Pass criteria**:
- First word is stack pointer (should be in 0x200xxxxx range)
- Second word is Reset_Handler address (should be 0x080xxxxx with thumb bit set)

### Step 3.4: Generate and verify hex file

```bash
arm-none-eabi-objcopy -O ihex ../test_results/test_minimal.elf ../test_results/test_minimal.hex
arm-none-eabi-objcopy -O binary ../test_results/test_minimal.elf ../test_results/test_minimal.bin
ls -la ../test_results/test_minimal.*
```

**Pass criteria**:
- .hex and .bin files generated
- .bin file starts at correct offset (can verify with hexdump)

---

## Phase 4: C Compilation Test

**Goal**: Verify HAL library C code compiles with GCC.

**Test directory**: `gcc/test/phase4_c_compile/`

### Step 4.1: Test HAL data types

Create `gcc/test/phase4_c_compile/test_types.c`:
```c
/* Test that MM32 type definitions work */
typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;
typedef signed char        s8;
typedef signed short       s16;
typedef signed int         s32;

#define __I  volatile const
#define __O  volatile
#define __IO volatile

__IO u32 test_reg;
__I  u32 readonly_reg;

int main(void) {
    u8 a = 0xFF;
    u16 b = 0xFFFF;
    u32 c = 0xFFFFFFFF;
    test_reg = c;
    return (int)(a + b + readonly_reg);
}
```

```bash
cd gcc/test/phase4_c_compile
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -c test_types.c -o ../test_results/test_types.o
```

**Pass criteria**: Compiles without warnings about type sizes.

### Step 4.2: Test a single HAL file

```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion

arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -c \
    -I Library/MM32SPIN05/HAL_Lib/Inc \
    -I RTE/Device/MM32SPIN06PF \
    Library/MM32SPIN05/HAL_Lib/Src/hal_gpio.c \
    -o gcc/test/test_results/hal_gpio.o
```

**Pass criteria**: Compiles. Note any warnings for later review.

### Step 4.3: Test all HAL files compile

```bash
for f in Library/MM32SPIN05/HAL_Lib/Src/*.c; do
    echo "Compiling: $f"
    arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Wall -Os -c \
        -I Library/MM32SPIN05/HAL_Lib/Inc \
        -I RTE/Device/MM32SPIN06PF \
        "$f" -o "gcc/test/test_results/$(basename $f .c).o" 2>&1 || echo "FAILED: $f"
done
```

**Pass criteria**: All 24 HAL files compile. Document any warnings.

---

## Phase 5: HAL Library Test

**Goal**: Verify HAL library links without missing symbols.

**Test directory**: `gcc/test/phase5_hal/`

### Step 5.1: Create HAL archive

```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion/gcc/test/test_results

arm-none-eabi-ar rcs libhal.a hal_*.o
arm-none-eabi-nm libhal.a | grep " T " | head -20
```

**Pass criteria**: Archive created with exported symbols.

### Step 5.2: Test linking HAL with startup

Create `gcc/test/phase5_hal/test_hal_link.c`:
```c
#include "hal_gpio.h"
#include "hal_rcc.h"

void SystemInit(void) {
    /* Minimal init */
}

int main(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = 0x01;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    while(1);
}
```

```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion/gcc/test/phase5_hal

arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -c \
    -I ../../../Library/MM32SPIN05/HAL_Lib/Inc \
    -I ../../../RTE/Device/MM32SPIN06PF \
    test_hal_link.c -o ../test_results/test_hal_link.o

arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -nostartfiles \
    -T ../phase3_linker/test_minimal.ld \
    ../test_results/test_startup.o ../test_results/test_hal_link.o \
    -L../test_results -lhal \
    -o ../test_results/test_hal_link.elf

arm-none-eabi-size ../test_results/test_hal_link.elf
```

**Pass criteria**: Links without undefined symbol errors.

---

## Phase 6: System Init Test

**Goal**: Verify system_mm32spin06xx_s.c compiles and provides SystemInit.

**Test directory**: `gcc/test/phase6_system/`

### Step 6.1: Compile system init

```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion

arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -c \
    -I Library/MM32SPIN05/HAL_Lib/Inc \
    -I RTE/Device/MM32SPIN06PF \
    RTE/Device/MM32SPIN06PF/system_mm32spin06xx_s.c \
    -o gcc/test/test_results/system_init.o
```

**Pass criteria**: Compiles without errors.

### Step 6.2: Verify SystemInit symbol

```bash
arm-none-eabi-nm gcc/test/test_results/system_init.o | grep SystemInit
```

**Pass criteria**: Shows `T SystemInit` (text/code symbol).

### Step 6.3: Check for Keil-specific constructs

```bash
grep -n "__asm\|__inline\|__attribute__\|#pragma" \
    RTE/Device/MM32SPIN06PF/system_mm32spin06xx_s.c \
    Library/MM32SPIN05/HAL_Lib/Src/*.c
```

**Document findings**: List any Keil-specific constructs that may need compatibility handling.

---

## Phase 7: Integration Build

**Goal**: Build complete firmware with GCC.

**Build directory**: `gcc/build/$(TARGET)/`

### Step 7.1: Create full GCC startup file

Create `gcc/startup_mm32spin05_gcc.s` with full vector table (as specified in port_hoverboard_hack.md).

Test assembly:
```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -c \
    gcc/startup_mm32spin05_gcc.s -o gcc/test/test_results/startup_full.o
arm-none-eabi-objdump -t gcc/test/test_results/startup_full.o | grep IRQHandler
```

**Pass criteria**: All IRQ handlers defined as weak symbols.

### Step 7.2: Create full linker script

Create `gcc/mm32spin06.ld` (as specified in port_hoverboard_hack.md).

### Step 7.3: Compile all application source

```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion
mkdir -p gcc/build/MM32SPIN06

for f in Src/*.c; do
    echo "Compiling: $f"
    arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -Wall -c \
        -I Inc \
        -I Library/MM32SPIN05/HAL_Lib/Inc \
        -I RTE/Device/MM32SPIN06PF \
        "$f" -o "gcc/build/MM32SPIN06/$(basename $f .c).o" 2>&1 || echo "FAILED: $f"
done
```

**Pass criteria**: All 17 application files compile.

### Step 7.4: Link complete firmware

```bash
arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb \
    -specs=nano.specs -specs=nosys.specs \
    -T gcc/mm32spin06.ld \
    -Wl,-Map=gcc/build/MM32SPIN06/firmware.map,--cref \
    -Wl,--gc-sections \
    gcc/test/test_results/startup_full.o \
    gcc/test/test_results/system_init.o \
    gcc/build/MM32SPIN06/main.o \
    gcc/build/MM32SPIN06/bldc.o \
    gcc/build/MM32SPIN06/calculation.o \
    gcc/build/MM32SPIN06/clarke.o \
    gcc/build/MM32SPIN06/delay.o \
    gcc/build/MM32SPIN06/FOC_Math.o \
    gcc/build/MM32SPIN06/hallhandle.o \
    gcc/build/MM32SPIN06/hardware.o \
    gcc/build/MM32SPIN06/initialize.o \
    gcc/build/MM32SPIN06/interrupt.o \
    gcc/build/MM32SPIN06/ipark.o \
    gcc/build/MM32SPIN06/park.o \
    gcc/build/MM32SPIN06/PID.o \
    gcc/build/MM32SPIN06/pwm_gen.o \
    gcc/build/MM32SPIN06/remoteUartBus.o \
    gcc/build/MM32SPIN06/sim_eeprom.o \
    gcc/build/MM32SPIN06/uart.o \
    -L gcc/test/test_results -lhal \
    -lm \
    -o gcc/build/MM32SPIN06/firmware.elf

arm-none-eabi-size gcc/build/MM32SPIN06/firmware.elf
```

**Pass criteria**:
- Links without errors
- Size fits in flash (<32KB text+data)

### Step 7.5: Create Makefile

Create `gcc/Makefile` (as specified in port_hoverboard_hack.md).

Test:
```bash
cd ~/src/Hoverboard-Firmware-Hack-Gen2.x-MM32-main/HoverBoardMindMotion/gcc
make clean
make TARGET=MM32SPIN06
```

**Pass criteria**: `make` produces identical output to manual commands.

---

## Phase 8: Binary Verification

**Goal**: Verify output binary is correctly formatted.

### Step 8.1: Check ELF structure

```bash
arm-none-eabi-readelf -h gcc/build/MM32SPIN06/firmware.elf
arm-none-eabi-readelf -S gcc/build/MM32SPIN06/firmware.elf
```

**Pass criteria**:
- Machine: ARM
- Entry point: 0x080xxxxx (in flash)
- Sections have correct addresses

### Step 8.2: Verify vector table

```bash
arm-none-eabi-objdump -s -j .isr_vector gcc/build/MM32SPIN06/firmware.elf | head -20
```

**Pass criteria**:
- Address 0x08000000: Stack pointer (0x200xxxxx)
- Address 0x08000004: Reset handler (0x080xxxxx, odd = thumb)
- Address 0x08000008: NMI handler
- Address 0x0800000C: HardFault handler

### Step 8.3: Check memory usage

```bash
arm-none-eabi-size -A gcc/build/MM32SPIN06/firmware.elf
```

**Pass criteria**:
- .text + .rodata + .data < 32KB
- .data + .bss < 4KB (or 8KB for SPIN25)

### Step 8.4: Compare with Keil output (if available)

If you have access to Keil-compiled .hex or .bin:

```bash
# Compare sizes
ls -la Objects/HoverboardOutputMM32SPIN06.hex gcc/build/MM32SPIN06/firmware.hex

# Compare vector tables (first 64 bytes)
xxd -l 64 Objects/HoverboardOutputMM32SPIN06.bin > gcc/test/test_results/keil_vectors.txt
xxd -l 64 gcc/build/MM32SPIN06/firmware.bin > gcc/test/test_results/gcc_vectors.txt
diff gcc/test/test_results/keil_vectors.txt gcc/test/test_results/gcc_vectors.txt
```

**Document differences**: Vector table layout should match. Entry points may differ slightly.

### Step 8.5: Generate final outputs

```bash
arm-none-eabi-objcopy -O ihex gcc/build/MM32SPIN06/firmware.elf gcc/build/MM32SPIN06/firmware.hex
arm-none-eabi-objcopy -O binary gcc/build/MM32SPIN06/firmware.elf gcc/build/MM32SPIN06/firmware.bin
```

**Pass criteria**: Files generated without errors.

---

## Phase 9: Hardware Validation

**Goal**: Verify firmware runs on actual hardware.

### Step 9.1: Flash firmware

Using Black Magic Probe:
```bash
arm-none-eabi-gdb -nx --batch \
    -ex 'target extended-remote /dev/cu.usbmodem*1' \
    -ex 'monitor swdp_scan' \
    -ex 'attach 1' \
    -ex 'load' \
    -ex 'compare-sections' \
    -ex 'kill' \
    gcc/build/MM32SPIN06/firmware.elf
```

Or via make:
```bash
cd gcc && make flash TARGET=MM32SPIN06
```

**Pass criteria**:
- Probe detects MM32 device
- Flash programming succeeds
- Verify shows no differences

### Step 9.2: Basic run test

```bash
arm-none-eabi-gdb -nx \
    -ex 'target extended-remote /dev/cu.usbmodem*1' \
    -ex 'monitor swdp_scan' \
    -ex 'attach 1' \
    -ex 'load' \
    -ex 'break main' \
    -ex 'continue' \
    gcc/build/MM32SPIN06/firmware.elf
```

**Pass criteria**: Execution reaches main().

### Step 9.3: Peripheral test

- Verify UART output (connect serial terminal)
- Verify LED blink (if applicable)
- Verify motor control response

**Pass criteria**: Same behavior as Keil-compiled firmware.

---

## Troubleshooting Checkpoints

### If Phase 2 fails (assembly)
- Check GNU AS syntax conversion
- Verify `.thumb` directive is present
- Check for ARM-mode instructions in Cortex-M0 code

### If Phase 3 fails (linking)
- Verify MEMORY regions match MCU
- Check ENTRY point exists
- Verify startup provides required symbols (_sdata, _edata, etc.)

### If Phase 4/5 fails (HAL compilation)
- Check include paths
- Look for Keil-specific pragmas
- May need to add `-Wno-*` flags for specific warnings

### If Phase 6 fails (system init)
- Check mm32_device.h is found
- Verify register definitions compile

### If Phase 7 fails (integration)
- Check for duplicate symbol definitions
- Verify interrupt handlers match vector table
- Check for missing `-lm` (math library)

### If Phase 9 fails (hardware)
- Verify clock configuration (72 MHz)
- Check vector table is at 0x08000000
- Verify stack pointer is valid

---

## Success Criteria Summary

| Phase | Key Verification |
|-------|------------------|
| 1 | `arm-none-eabi-gcc --version` works |
| 2 | Startup .o file has correct sections |
| 3 | ELF vector table at 0x08000000 |
| 4 | All HAL .c files compile |
| 5 | HAL links without undefined symbols |
| 6 | SystemInit symbol exported |
| 7 | Complete firmware links, <32KB |
| 8 | Binary structure matches expected layout |
| 9 | Runs on hardware, matches Keil behavior |

---

## File Checklist

Files to create (in order):

### Directory Structure
- [ ] `gcc/` directory
- [ ] `gcc/test/` directory
- [ ] `gcc/test/test_results/` directory (gitignore contents)
- [ ] `gcc/test/phase1_toolchain/`
- [ ] `gcc/test/phase2_assembly/`
- [ ] `gcc/test/phase3_linker/`
- [ ] `gcc/test/phase4_c_compile/`
- [ ] `gcc/test/phase5_hal/`
- [ ] `gcc/test/phase6_system/`
- [ ] `gcc/build/` directory (gitignore contents)

### Test Files (Phases 1-6)
- [ ] `gcc/test/phase1_toolchain/test_cortexm0.c` (Phase 1)
- [ ] `gcc/test/phase2_assembly/test_startup_minimal.s` (Phase 2)
- [ ] `gcc/test/phase2_assembly/test_override.c` (Phase 2)
- [ ] `gcc/test/phase3_linker/test_minimal.ld` (Phase 3)
- [ ] `gcc/test/phase4_c_compile/test_types.c` (Phase 4)
- [ ] `gcc/test/phase5_hal/test_hal_link.c` (Phase 5)
- [ ] `gcc/test/run_tests.sh` (test runner script)

### Production Files (Phase 7)
- [ ] `gcc/startup_mm32spin05_gcc.s`
- [ ] `gcc/startup_mm32spin25_gcc.s`
- [ ] `gcc/mm32spin05.ld`
- [ ] `gcc/mm32spin06.ld`
- [ ] `gcc/mm32spin25.ld`
- [ ] `gcc/Makefile`
- [ ] `gcc/toolchain.mk` (optional, for shared flags)
- [ ] `gcc/gcc_compat.h` (if needed)

### Git Configuration
- [ ] `gcc/.gitignore` (ignore build/ and test/test_results/)

---

## Appendix A: Sample Makefile with Test Targets

This Makefile skeleton shows how to implement the `make check-*` targets:

```makefile
# gcc/Makefile - Build and test targets for GCC port

#------------------------------------------------------------------------------
# Configuration
#------------------------------------------------------------------------------
TARGET ?= MM32SPIN06
PREFIX ?= arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
AR      = $(PREFIX)ar
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size
NM      = $(PREFIX)nm
READELF = $(PREFIX)readelf

# Directories
ROOT_DIR    = ..
BUILD_DIR   = build/$(TARGET)
TEST_DIR    = test
RESULTS_DIR = $(TEST_DIR)/test_results

# Common flags
MCU = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft

#------------------------------------------------------------------------------
# Default target
#------------------------------------------------------------------------------
.PHONY: all
all: $(BUILD_DIR)/firmware.elf $(BUILD_DIR)/firmware.hex $(BUILD_DIR)/firmware.bin
	$(SIZE) $(BUILD_DIR)/firmware.elf

#------------------------------------------------------------------------------
# Test targets
#------------------------------------------------------------------------------
.PHONY: test check
test: check
check: check-env check-asm check-ld check-compile check-hal check-system check-build check-binary
	@echo "========================================"
	@echo "All checks passed!"
	@echo "========================================"

# Phase 1: Toolchain verification
.PHONY: check-env
check-env:
	@echo "=== Phase 1: Checking toolchain ==="
	@which $(CC) > /dev/null || (echo "ERROR: $(CC) not found" && exit 1)
	@$(CC) --version | head -1
	@echo "Testing Cortex-M0 compilation..."
	@mkdir -p $(RESULTS_DIR)
	@$(CC) $(MCU) -c $(TEST_DIR)/phase1_toolchain/test_cortexm0.c \
		-o $(RESULTS_DIR)/test_cortexm0.o
	@$(CC) $(MCU) --specs=nano.specs --specs=nosys.specs -c \
		$(TEST_DIR)/phase1_toolchain/test_cortexm0.c \
		-o $(RESULTS_DIR)/test_cortexm0_nano.o
	@echo "Phase 1: PASS"

# Phase 2: Assembly syntax
.PHONY: check-asm
check-asm:
	@echo "=== Phase 2: Testing assembly syntax ==="
	@mkdir -p $(RESULTS_DIR)
	@$(AS) $(MCU) -c $(TEST_DIR)/phase2_assembly/test_startup_minimal.s \
		-o $(RESULTS_DIR)/test_startup.o
	@$(OBJDUMP) -h $(RESULTS_DIR)/test_startup.o | grep -q ".isr_vector" || \
		(echo "ERROR: .isr_vector section not found" && exit 1)
	@$(NM) $(RESULTS_DIR)/test_startup.o | grep -q "Reset_Handler" || \
		(echo "ERROR: Reset_Handler not found" && exit 1)
	@echo "Testing weak symbol override..."
	@$(CC) $(MCU) -c $(TEST_DIR)/phase2_assembly/test_override.c \
		-o $(RESULTS_DIR)/test_override.o
	@echo "Phase 2: PASS"

# Phase 3: Linker script
.PHONY: check-ld
check-ld: check-asm
	@echo "=== Phase 3: Testing linker script ==="
	@$(CC) $(MCU) -nostartfiles \
		-T $(TEST_DIR)/phase3_linker/test_minimal.ld \
		$(RESULTS_DIR)/test_startup.o \
		-o $(RESULTS_DIR)/test_minimal.elf
	@$(OBJDUMP) -h $(RESULTS_DIR)/test_minimal.elf | grep ".isr_vector" | \
		grep -q "08000000" || (echo "ERROR: Vector table not at 0x08000000" && exit 1)
	@$(OBJCOPY) -O ihex $(RESULTS_DIR)/test_minimal.elf $(RESULTS_DIR)/test_minimal.hex
	@$(OBJCOPY) -O binary $(RESULTS_DIR)/test_minimal.elf $(RESULTS_DIR)/test_minimal.bin
	@echo "Phase 3: PASS"

# Phase 4: C compilation
.PHONY: check-compile
check-compile:
	@echo "=== Phase 4: Testing C compilation ==="
	@$(CC) $(MCU) -Wall -c $(TEST_DIR)/phase4_c_compile/test_types.c \
		-o $(RESULTS_DIR)/test_types.o
	@echo "Compiling HAL files..."
	@for f in $(ROOT_DIR)/Library/MM32SPIN05/HAL_Lib/Src/*.c; do \
		$(CC) $(MCU) -Wall -Os -c \
			-I $(ROOT_DIR)/Library/MM32SPIN05/HAL_Lib/Inc \
			-I $(ROOT_DIR)/RTE/Device/MM32SPIN06PF \
			"$$f" -o "$(RESULTS_DIR)/$$(basename $$f .c).o" || exit 1; \
	done
	@echo "Phase 4: PASS"

# Phase 5: HAL library
.PHONY: check-hal
check-hal: check-compile check-asm
	@echo "=== Phase 5: Testing HAL library ==="
	@$(AR) rcs $(RESULTS_DIR)/libhal.a $(RESULTS_DIR)/hal_*.o
	@$(CC) $(MCU) -Os -c \
		-I $(ROOT_DIR)/Library/MM32SPIN05/HAL_Lib/Inc \
		-I $(ROOT_DIR)/RTE/Device/MM32SPIN06PF \
		$(TEST_DIR)/phase5_hal/test_hal_link.c \
		-o $(RESULTS_DIR)/test_hal_link.o
	@$(CC) $(MCU) -nostartfiles \
		-T $(TEST_DIR)/phase3_linker/test_minimal.ld \
		$(RESULTS_DIR)/test_startup.o \
		$(RESULTS_DIR)/test_hal_link.o \
		-L$(RESULTS_DIR) -lhal \
		-o $(RESULTS_DIR)/test_hal_link.elf
	@echo "Phase 5: PASS"

# Phase 6: System init
.PHONY: check-system
check-system:
	@echo "=== Phase 6: Testing system init ==="
	@$(CC) $(MCU) -Os -c \
		-I $(ROOT_DIR)/Library/MM32SPIN05/HAL_Lib/Inc \
		-I $(ROOT_DIR)/RTE/Device/MM32SPIN06PF \
		$(ROOT_DIR)/RTE/Device/MM32SPIN06PF/system_mm32spin06xx_s.c \
		-o $(RESULTS_DIR)/system_init.o
	@$(NM) $(RESULTS_DIR)/system_init.o | grep -q "T SystemInit" || \
		(echo "ERROR: SystemInit not exported" && exit 1)
	@echo "Phase 6: PASS"

# Phase 7: Integration build (uses main build targets)
.PHONY: check-build
check-build: $(BUILD_DIR)/firmware.elf
	@echo "=== Phase 7: Integration build ==="
	@$(SIZE) $(BUILD_DIR)/firmware.elf
	@echo "Phase 7: PASS"

# Phase 8: Binary verification
.PHONY: check-binary
check-binary: $(BUILD_DIR)/firmware.elf
	@echo "=== Phase 8: Binary verification ==="
	@$(READELF) -h $(BUILD_DIR)/firmware.elf | grep -q "ARM" || \
		(echo "ERROR: Not an ARM binary" && exit 1)
	@$(OBJDUMP) -s -j .isr_vector $(BUILD_DIR)/firmware.elf | head -5
	@$(SIZE) -A $(BUILD_DIR)/firmware.elf
	@echo "Phase 8: PASS"

#------------------------------------------------------------------------------
# Clean
#------------------------------------------------------------------------------
.PHONY: clean clean-tests
clean:
	rm -rf build

clean-tests:
	rm -rf $(RESULTS_DIR)/*

clean-all: clean clean-tests

#------------------------------------------------------------------------------
# Flash (Phase 9)
#------------------------------------------------------------------------------
.PHONY: flash
flash: $(BUILD_DIR)/firmware.elf
	arm-none-eabi-gdb -nx --batch \
		-ex 'target extended-remote /dev/cu.usbmodem*1' \
		-ex 'monitor swdp_scan' \
		-ex 'attach 1' \
		-ex 'load' \
		-ex 'compare-sections' \
		-ex 'kill' \
		$(BUILD_DIR)/firmware.elf

#------------------------------------------------------------------------------
# Build rules (abbreviated - see port_hoverboard_hack.md for full version)
#------------------------------------------------------------------------------
# ... (firmware build rules here)
```

---

## Appendix B: Test Runner Script

`gcc/test/run_tests.sh`:

```bash
#!/bin/bash
# Run all tests and report results
set -e

cd "$(dirname "$0")/.."

echo "Running GCC port tests..."
echo ""

make check-env && echo ""
make check-asm && echo ""
make check-ld && echo ""
make check-compile && echo ""
make check-hal && echo ""
make check-system && echo ""
make check-build && echo ""
make check-binary && echo ""

echo ""
echo "========================================"
echo "All tests passed!"
echo "========================================"
```

Make executable: `chmod +x gcc/test/run_tests.sh`
