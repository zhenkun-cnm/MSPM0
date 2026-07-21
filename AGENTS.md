# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

FreeRTOS-based multi-sensor firmware for the **TI MSPM0G3507** (ARM Cortex-M0+, LP-MSPM0G3507 LaunchPad). Built with **Keil MDK (uVision V5.4, ARM Compiler 6 / armclang)** using the TI MSPM0 SDK + SysConfig. The codebase is written for a four-layer decoupled architecture (App → Device → Port → HAL).

## Build & Flash

There is no command-line build script; the project is driven by the Keil `.uvprojx`. Build headlessly with UV4:

```sh
# Build (rebuild target)
"D:/Keil/V5.4/UV4/UV4.exe" -b "c:/ti/mspm0_project/board/empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx" -o build.log

# Clean rebuild
"D:/Keil/V5.4/UV4/UV4.exe" -r "c:/ti/mspm0_project/board/empty/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx" -o build.log
```

UV4 runs asynchronously and returns its result via the exit code; **read `build.log` for errors/warnings** after it finishes (it does not print to stdout). Build artifacts (`Objects/`, `*.lst`, `*.map`, `.vscode/`) are git-ignored.

- **Toolchain / SDK paths** (hard-coded in `c_cpp_properties.json` and the `.uvprojx`): Keil at `D:\Keil\V5.4`, MSPM0 SDK at `c:\ti\mspm0_sdk_2_10_00_04`.
- **Adding a new source file requires editing the `.uvprojx`** (add the `<File>` entry to the right group) — the IntelliSense `c_cpp_properties.json` does NOT control what Keil compiles. New `app_*.c` / `port_*.c` / `dev_*.c` files will be silently excluded from the build until registered in the project.
- **Device config (clocks/GPIO/peripherals) is generated from `empty.syscfg`** via SysConfig into `ti_msp_dl_config.c/h`. Do not hand-edit the generated files; change pin/peripheral assignments in `.syscfg` and regenerate. `empty.c` is the SDK entry stub holding `main()`.

There are no unit tests — verification is on-target (flash + observe UART log over the debug serial port).

## Architecture: Four-Layer Decoupling

```
APP    (APP/src, APP/inc)              FreeRTOS tasks, business logic
  │    only calls Device interfaces + FreeRTOS API. NO register / DL_* access.
Device (Device/inc — headers only)    OOP interface contracts (vtables)
  │    pure .h. defines struct-of-function-pointers + a GetXxx() factory.
Port   (board/empty/port/src+inc)      hardware drivers implementing Device contracts
  │    owns register access, DL_* DriverLib calls, bit-bang SPI, may include ti_msp_dl_config.h
HAL    (board/empty/ti_msp_dl_config*) TI SysConfig-generated peripheral init
```

The strict include rule is the heart of this codebase — **App-layer files must not include `ti_msp_dl_config.h` or call `DL_GPIO_*`/register code.** They interact with hardware only through Device handles. Preserve this when adding features.

### Device-layer OOP pattern

Each device is a `struct` of function pointers (a manual vtable) plus a single global factory `GetXxx()` returning a static singleton instance. Example: `dev_led.h` defines `DevLED { init, on, off, toggle, getState }`; `port_led.c` defines static implementations, wires them into a static `DevLED`, and `GetLED()` returns it. App code does:

```c
DevLED *led = GetLED();
led->init(led);
led->toggle(led);
```

When adding a device: declare the contract in `Device/inc/dev_<x>.h`, implement it + `Get<X>()` in `board/empty/port/src/port_<x>.c`, and consume it from `APP/src/app_<x>.c`. The `self` pointer is passed to every method even though current implementations ignore it (singletons).

### Boot flow

`main()` (`empty.c`) → `PORT_SYSTEM_Init()` (wraps `SYSCFG_DL_init()`, must run before scheduler) → `app_init()` (creates `start_task`) → `app_start()` (`vTaskStartScheduler()`). `start_task` (in `app_init.c`) creates all worker tasks and any shared queues inside a critical section, logs the boot banner, then `vTaskDelete(NULL)`s itself. Register new tasks here.

### FreeRTOS

heap_4 memory manager. Config in `board/empty/FreeRTOSConfig.h`. Task stack sizes are in **words** (4 bytes each), typically 128–512. The port is `Middleware/FreeRTOS/portable/GCC/ARM_CM0` (GCC CM0 port used under armclang).

### Logging

Use the `LOG_*` macros from `port_log.h` (`LOG_RAW`/`LOG_ERROR`/`LOG_INFO`/`LOG_DEBUG`) — never raw `printf`. Output goes over UART. Compile-time `LOG_LEVEL` (default 3 = INFO) gates which levels emit code. Messages need explicit `\r\n`.

## Hardware Notes (verify against `empty.syscfg` before trusting)

- **LED**: PB22 (`DL_GPIO`).
- **EC11 encoder**: PA24 (A/CLK), PA25 (B/DT), PA26 (button/SW). Driver in `port_encoder.c` does Gray-code decode + software debounce + a 5-state button FSM (short/long/double-press), GPIO-polled on a 5 ms task tick.
- **W25Q128 Flash**: hardware **SPI1** (CS in `ti_msp_dl_config`). Full JEDEC ID / 4KB sector erase / page program / read.
- **ST7735 TFT**: shares hardware **SPI1** with the Flash (PA17 SCK, PA18 MOSI), plus PB13 RESET, PB12 DC, PB11 CS, PB10 BLK.
- **ICM-20948 IMU**: **software bit-bang SPI** (PB9 SCK, PB8 MOSI, PB7 MISO, PB6 CS), CPOL=0/CPHA=0, ~150 kHz. Bank-switched registers; WHO_AM_I must read 0xEA. SPI clock is capped (~400 kHz) by an LSF0108 level shifter's RC edges — do not raise the bit-bang frequency. See `c:\ti\mspm0_project\ICM-20948.datasheet.md` for the datasheet cache before re-deriving register details.
- **8-channel grayscale line sensor**: `Find_Block` uses AD0=PA8, AD1=PB5, AD2=PA9 as mux select outputs and OUT=PB4 as input. The leftmost physical sensor is channel 1; channel numbers increase left-to-right from the car's perspective.
- **Motor encoder calibration for speed/distance math**: theoretical value is 1040 pulses per wheel revolution, but the user-measured calibration is 1054 pulses per wheel revolution for both left and right wheels. Treat 1054 as the user-confirmed value for converting encoder counts to speed/distance.

## Working Docs

`task_plan.md`, `findings.md`, `progress.md` (root + a copy under `board/empty/keil`) are the running planning/status notes for this effort and are kept current — consult them for module-completion status and decision history.
