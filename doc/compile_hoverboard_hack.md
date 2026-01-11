# Compiling Hoverboard-Firmware-Hack-Gen2.x-MM32 on M1 Mac

Research date: January 2025

## Summary

**Yes, it is possible to compile this firmware on an M1 Mac Mini using open source tools**, but it requires additional work to port the project from Keil MDK to a GCC/Makefile build system. The project currently only supports the Keil IDE with no Makefile provided.

## Project Overview

The firmware targets **MindMotion MM32SPIN series** motor control microcontrollers:

| MCU | Core | SRAM | Flash |
|-----|------|------|-------|
| MM32SPIN05PF | ARM Cortex-M0 | 4 KB | 8 KB |
| MM32SPIN06PF | ARM Cortex-M0 | 4 KB | 16 KB |
| MM32SPIN25PF | ARM Cortex-M0 | 8 KB | 8 KB |

System clock: 72 MHz (from 12 MHz HSI via PLL)

## Current Build System (Keil MDK)

The project uses **Keil MDK-ARM** with:
- Compiler: ARMCLANG v6.19 (LLVM-based)
- Project file: `HoverBoardMindMotion/Hoverboard.uvprojx`
- Device packs: MindMotion MM32SPIN0x_DFP and MM32SPIN2x_DFP
- CMSIS 5.9.0

**Keil MDK is a commercial product** (Professional ~$4,900 USD), but there is a free [MDK-Community Edition](https://www.keil.arm.com/mdk-community/) for non-commercial use with no code size limits. However, the traditional μVision IDE is **Windows-only**. The newer Keil Studio supports macOS but is cloud-based.

## Open Source Toolchain for M1 Mac

### ARM GCC Compiler (Native Apple Silicon)

ARM provides official native Apple Silicon builds of the GNU toolchain:

```bash
# Download and install ARM GNU Toolchain 14.2
wget https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.pkg
sudo installer -pkg arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.pkg -target /

# Add to PATH
echo '/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin' | sudo tee -a /etc/paths
```

Alternative installation methods:
- **Homebrew**: `brew install --cask gcc-arm-embedded`
- **MacPorts**: `sudo port install arm-none-eabi-gcc`
- **xPack**: Has [pre-built packages for M1](https://gist.github.com/mandrean/0194f6a05727161db74da1d6fffc075b)

### MM32 GCC Support

The [iclite/mm32_startup](https://github.com/iclite/mm32_startup) repository provides:
- Startup files compatible with GCC (not just Keil)
- Header files for MM32 devices
- Clock initialization files
- MIT licensed

This is the key resource needed to port the firmware to GCC.

## Porting Requirements

To compile on M1 Mac with open source tools, you would need to:

### 1. Create a Makefile

The project has no Makefile. You need to create one that:
- Sets compiler flags for Cortex-M0: `-mcpu=cortex-m0 -mthumb -mfloat-abi=soft`
- Defines include paths for HAL library and CMSIS
- Links with the correct linker script
- Handles startup assembly files

Example compiler flags:
```makefile
CFLAGS = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft
CFLAGS += -Os -ffunction-sections -fdata-sections
CFLAGS += -Wall -std=c99
LDFLAGS = -T linker_script.ld -nostartfiles -Wl,--gc-sections
```

### 2. Port Startup Files

Replace Keil-specific startup assembly (`startup_mm32spin06xx_s_keil.s`) with GCC-compatible versions from [iclite/mm32_startup](https://github.com/iclite/mm32_startup) or write new ones.

### 3. Create Linker Script

Keil auto-generates scatter files. For GCC, create a `.ld` linker script:

```ld
/* Example for MM32SPIN05PF */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 8K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 4K
}
```

### 4. Handle Assembly Syntax Differences

Keil uses ARM assembler syntax; GCC uses GNU assembler syntax. Some assembly files may need conversion.

### 5. Verify HAL Compatibility

The MindMotion HAL library in the project should work with GCC, but may have Keil-specific pragmas or intrinsics that need adjustment.

## Programming/Flashing on macOS

### Black Magic Probe (Recommended)

[Black Magic Debug](https://black-magic.org/hacking/target-mm32.html) has native MM32 support. It's open source and works on macOS.

### CMSIS-DAP

MindMotion provides their own [MM32-DAP](https://mindmotion.com.cn/en/support/development_tools/debug_and_programming_tools/mm32_dap/) debugger with open-source firmware.

### OpenOCD

OpenOCD does **not** have native MM32 support. MM32 flash operations differ from STM32:
- On STM32: 16-bit writes use bits 0:15 for even halfwords, 16:31 for odd
- On MM32 Cortex-M0: 16-bit writes always use bits 0:15

You would need a custom OpenOCD fork or use Black Magic Probe instead.

## Effort Estimate

| Task | Complexity |
|------|------------|
| Install ARM GCC toolchain | Simple |
| Create Makefile build system | Medium |
| Port startup files to GCC | Medium |
| Create linker scripts (3 variants) | Simple |
| Test and debug HAL compatibility | Medium |
| Set up flashing workflow | Simple |

## Recommendation

**For hobby/learning purposes**: Port to GCC. Use the [iclite/mm32_startup](https://github.com/iclite/mm32_startup) repo as reference, and look at [opensource-toolchain-stm32](https://github.com/cjacker/opensource-toolchain-stm32) for Makefile patterns.

**For quick results**: Use Keil MDK Community Edition on a Windows VM (Parallels/UTM work well on M1). The Community Edition is free for non-commercial use with no code size limits.

**Alternative**: The project provides [pre-compiled binaries](https://github.com/AILIFE4798/Hoverboard-Firmware-Hack-Gen2.x-MM32), so you may not need to compile at all unless you're making modifications.

## Sources

- [ARM GNU Toolchain Installation Guide](https://learn.arm.com/install-guides/gcc/arm-gnu/)
- [xPack GNU Arm Embedded GCC for M1](https://gist.github.com/mandrean/0194f6a05727161db74da1d6fffc075b)
- [iclite/mm32_startup - GCC-compatible startup files](https://github.com/iclite/mm32_startup)
- [opensource-toolchain-stm32 (applies to MM32)](https://github.com/cjacker/opensource-toolchain-stm32)
- [Black Magic Debug MM32 Support](https://black-magic.org/hacking/target-mm32.html)
- [Keil MDK Community Edition](https://www.keil.arm.com/mdk-community/)
- [Porting Keil to GNU Toolchain (Renesas)](https://www.renesas.com/en/document/apn/b-024-porting-keil-uvision-project-gnu-chain-tool)
