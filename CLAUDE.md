# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

Requires `arm-none-eabi-gcc`, CMake 3.22+, and Ninja.

```bash
# Configure (once)
cmake --preset Debug

# Build
cmake --build build/Debug

# Flash (requires st-flash)
st-flash write build/Debug/dexslide_stm32.bin 0x08000000
```

Output artifacts: `build/Debug/dexslide_stm32.elf`, `.bin`, `.map`.

## Project Status

CubeMX-generated skeleton with application logic implemented. Main loop reads 5 ADS1115 ADCs (20 channels), packs into USB CDC frames, and displays on debug LCD.

**Implemented drivers:**
- `Core/Src/ads1115.c/h` — ADS1115 I2C driver (single-shot, 860 SPS, 4 channels per device)
- `Core/Src/usb_comm.c/h` — Frame packing + USB CDC transmit (`[0xAA55][20×uint16][XOR checksum][0x0D]`)
- `Core/Src/lcd_driver.c/h` — Debug LCD display of joint angles
- `Core/Src/main.c` — ADC polling loop: read 5 ADCs × 4 channels, pack, send

## Hardware Configuration

**MCU:** STM32F103RCT6 — Cortex-M3, 72 MHz (8 MHz HSE × PLL9), 256 KB Flash, 48 KB RAM.

**Peripherals configured in CubeMX (`dexslide_stm32.ioc`):**

| Peripheral | Pins | Config | Purpose |
|---|---|---|---|
| I2C1 | PB8/PB9 (remapped) | 100 kHz, 7-bit | Thumb (0x48), Index (0x49), Middle (0x4A) ADCs |
| I2C2 | PB10/PB11 | 100 kHz, 7-bit | Ring (0x48), Pinky (0x49) ADCs |
| USART1 | PA9/PA10 | 115200 8N1 | Debug serial |
| USB FS | PA11/PA12 | CDC, VID 0x0483 PID 0x5740 | Data stream to PC |
| GPIO | PA8 | Push-pull output | LED/debug indicator |
| SWD | PA13/PA14 | JTAG disabled, SWD only | Debug probe |

## Code Organization

- `Core/Src/main.c` — Peripheral init + main loop. Add application code in `USER CODE` blocks.
- `Core/Src/stm32f1xx_hal_msp.c` — GPIO/clock setup for I2C, UART (CubeMX-managed).
- `USB_DEVICE/App/usbd_cdc_if.c` — CDC callbacks. `CDC_Transmit_FS()` sends data over USB (non-blocking, returns `USBD_BUSY` if prior TX pending).
- `USB_DEVICE/App/usbd_desc.c` — USB descriptors.
- `Drivers/` and `Middlewares/` — STM32 HAL and USB Device Library (do not edit).
- `cmake/stm32cubemx/CMakeLists.txt` — CubeMX-generated source lists. When adding new `.c` files, add them to `CMakeLists.txt` (top-level) under `target_sources`, not the CubeMX-generated one.

## CubeMX Workflow

The `.ioc` file is the CubeMX project. Re-generating from CubeMX will overwrite code **outside** `USER CODE BEGIN/END` blocks. All hand-written code must go inside those blocks. The HAL MSP file (`stm32f1xx_hal_msp.c`) and interrupt file (`stm32f1xx_it.c`) follow the same convention.

## Key Globals

Peripheral handles declared in `main.c`, used throughout:
- `hi2c1`, `hi2c2` — I2C bus handles
- `huart1` — USART1 handle
- `hpcd_USB_FS` — USB PCD handle (declared in `usbd_conf.c`)

## Memory Constraints

Linker script (`STM32F103XX_FLASH.ld`): heap 512 bytes, stack 1024 bytes. Increase these if adding significant dynamic allocation or deep call chains.
