# MSPM0G3507 FreeRTOS Multi-Sensor Firmware

Firmware project for the TI MSPM0G3507 LaunchPad. The project uses FreeRTOS and a four-layer architecture that keeps application logic, device contracts, board ports, and generated hardware configuration separate.

## Target

- MCU: TI MSPM0G3507, ARM Cortex-M0+
- Board: LP-MSPM0G3507 LaunchPad
- IDE: Keil MDK uVision 5.4
- Compiler: ARM Compiler 6 / armclang
- SDK: TI MSPM0 SDK 2.10.00.04
- RTOS: FreeRTOS with `heap_4`

## Architecture

```text
APP     FreeRTOS tasks and business logic
Device  Header-only device interfaces using function-pointer structs
Port    Board drivers implementing Device contracts
HAL     SysConfig-generated TI DriverLib configuration
```

Application files should use Device interfaces and FreeRTOS APIs only. Hardware register access, `DL_*` DriverLib calls, and `ti_msp_dl_config.h` belong in the Port/HAL layers.

## Main Modules

- LED task and board LED driver
- EC11 encoder polling driver with rotation, short press, long press, and double press events
- W25Q128 SPI flash driver with JEDEC ID, sector erase, page program, and read
- ST7735 TFT driver over shared SPI1
- ICM-20948 IMU driver over bit-banged SPI and an application-level EKF attitude task
- UART logging through `LOG_*` macros

## Build

The project is built through the Keil project file:

```sh
"D:/Keil/V5.4/UV4/UV4.exe" -b "c:/ti/mspm0_project/board/empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx" -o build.log
```

For a clean rebuild:

```sh
"D:/Keil/V5.4/UV4/UV4.exe" -r "c:/ti/mspm0_project/board/empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx" -o build.log
```

After building, inspect `build.log` because UV4 writes diagnostics there instead of standard output.

## Notes For Development

- Add new source files to the `.uvprojx`; IntelliSense settings do not control the Keil build.
- Change generated peripheral setup in `empty.syscfg`, then regenerate `ti_msp_dl_config.c/h`.
- Use `LOG_RAW`, `LOG_ERROR`, `LOG_INFO`, or `LOG_DEBUG` for logging instead of raw `printf`.
- Task stack sizes in FreeRTOS are specified in words, not bytes.
- On-target verification is expected after flashing; there are no host-side unit tests in this repository.

## Working Documents

- `task_plan.md`
- `findings.md`
- `progress.md`
- `AGENTS.md`
