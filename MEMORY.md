# Codex Memory — MSPM0 Project

> **Ownership:** This file is maintained by Codex for continuity between tasks. The user is not expected to edit or manage it. Update it only when a durable project fact, decision, workflow rule, or unresolved risk changes.

## Project identity

- Firmware for the TI MSPM0G3507 LaunchPad, built with Keil MDK / ARM Compiler 6 and FreeRTOS.
- Repository root: `C:\ti\mspm0_project`.
- Primary Keil project: `board\empty\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx`.
- Existing planning records: `task_plan.md`, `findings.md`, and `progress.md` (also mirrored under `board\empty\keil`). Consult them when task-specific history is needed.

## Non-negotiable architecture

- Preserve the four layers: **APP → Device → Port → HAL**.
- APP files may use Device interfaces and FreeRTOS only. They must not include `ti_msp_dl_config.h` or call `DL_*` / registers.
- Device is header-only interface contracts using function-pointer vtables and `GetXxx()` singleton factories.
- Port implements Device contracts and owns DriverLib, register, and GPIO/SPI access.
- `empty.syscfg` is authoritative for `ti_msp_dl_config.c/h`. Direct edits to generated files are permitted only as a controlled temporary aid: immediately apply the identical change in the graphical SysConfig editor, save/regenerate, and verify the generated output matches. Never retain a generated-file-only change.
- When adding a C source file, add it to the `.uvprojx`; otherwise Keil silently excludes it.

## Boot and verification

- Startup: `main()` → `PORT_SYSTEM_Init()` → `app_init()` → `app_start()`; `start_task` creates worker tasks then deletes itself.
- Use `LOG_*` macros from `port_log.h`, with explicit `\r\n`; do not use raw `printf`.
- There are no automated unit tests. Preferred verification is a Keil rebuild plus on-target UART observation.
- The Keil build log, not console output, is authoritative for errors and warnings.

## Hardware facts that must be retained

- LED: PB22.
- EC11: PA24/PA25/PA26; polled every 5 ms.
- W25Q128 Flash and ST7735 TFT share hardware SPI1; their chip selects must remain mutually exclusive.
- TFT control: PB13 RESET, PB12 DC, PB11 CS, PB10 backlight.
- ICM-20948 uses bit-banged SPI on PB9/PB8/PB7/PB6, mode 0. Keep it at about 150 kHz; do not raise it above the LSF0108-limited safe range.
- Line sensor mux: PA8, PB5, PA9; output PB4. Physical channel order is left-to-right, numbered 1–8 from the car perspective.
- Motor encoder conversion uses the user-calibrated value: **1054 pulses per wheel revolution** for both wheels.

## Current durable implementation state

- INS, UART command reception, motion control, TFT PID menu, and navigation v1 have previously been restored and build-verified.
- Navigation v1 lives in `app_nav`; it coordinates proven motion primitives using INS pose. `nav square` is an action sequence (drive/turn), not ideal fixed waypoints.
- Arc v1 belongs in `app_motion`: differential wheel-speed inner loops with IMU yaw used as the stop condition. Do not add a path/yaw outer PID without a new requirement.
- `app_stack_monitor.c` is intentionally excluded because the current FreeRTOS configuration does not declare `xTaskGetHandle()`.

## Maintenance policy

- Before significant firmware work, read this file and the relevant current entries in `task_plan.md`, `findings.md`, and `progress.md`.
- After completing meaningful work, update this file only with facts that will matter across future tasks; keep transient task details in the existing planning files.
- If this file conflicts with source code, `AGENTS.md`, generated configuration, or a verified build result, treat the latter as authoritative and correct this file.
