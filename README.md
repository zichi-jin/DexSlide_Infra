# dexslide_infra

STM32 firmware and board bring-up tools for DexSlide.

This repository owns the lower-level infrastructure for the data glove:

- STM32F103 firmware for reading ADS1115 ADC boards.
- USB CDC serial streaming from the glove controller to the PC.
- CubeMX/CMake/toolchain files, startup code, linker script, HAL drivers, and USB middleware.
- Hardware-facing calibration and monitoring scripts.
- PCB notes, pinout docs, and chip datasheets that were previously in the main repository.

The PC-side reconstruction, skeleton calibration, visualization, and retargeting code lives in the main `dexslide` repository.

## Build

```bash
cmake --preset Debug
cmake --build build/Debug
```

Output artifacts are written under `build/Debug/`.

## Flash

```bash
st-flash write build/Debug/dexslide_stm32.bin 0x08000000
```

## Calibrate Glove Angles

Run the calibration script from this repository, but write the result into the main `dexslide` repository so the PC-side tools can read it by default:

```bash
python scripts/glove_calibrate.py --port /dev/ttyACM0 --out ../DexSlide/assets/calibration/glove_calibration.json
```

Verify the angle mapping:

```bash
python scripts/ads_live_monitor.py --port /dev/ttyACM0 --angles --calib-file ../DexSlide/assets/calibration/glove_calibration.json
```

## Important Layout Notes

- `Core/`, `USB_DEVICE/`, `Drivers/`, `Middlewares/`, `cmake/`, `startup_stm32f103xe.s`, and `STM32F103XX_FLASH.ld` are one CubeMX/CMake project.
- The old outer `firmware/Core` template from the previous monorepo was intentionally not copied here. It was not the active build.
- Keep the CubeMX `.ioc` file in this repository when it is restored or regenerated. It is source, not build output.
