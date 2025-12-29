# Copilot / AI Agent Instructions — VXL (ESP32)

Quick, actionable guidance to edit and extend this repository safely and productively.

## Big picture
- Project is an ESP-IDF C application (ESP32) implementing a closed-loop humidity/temperature chamber controller. See `README.md` and `main/VXL.c`.
- Main responsibilities:
  - Sensors: SHT35 (I2C at 0x44) for ambient T/H (`sht_read()`), DS18B20 (OneWire on GPIO4) for water temp (`ds18b20_read()`).
  - Actuators: Peltier, pump, blower, case fan controlled via GPIO/PWM. Pins defined as `PIN_*` macros at top of `main/VXL.c`.
  - Tasks: `task_control` (logic + watchdog), `task_ui` (LCD + buttons), `task_soft_start` (soft PWM ramp).
  - Data logging: writes `log.csv` to `/sdcard` using FAT VFS (`sd_init()` and `log_to_sd()`).

## Build / dev flow (must-follow)
- ESP-IDF is required (project `sdkconfig` indicates ESP-IDF v5.x). Use `idf.py` from your ESP-IDF environment.
- Typical sequence:
  - `idf.py set-target esp32`
  - `idf.py menuconfig` (if you need to edit build options)
  - `idf.py build`
  - `idf.py -p <PORT> flash monitor` (Windows use `COMx` as `<PORT>`)
- Common gotchas: SD SPI is run at reduced frequency in `sd_init()` (`h.max_freq_khz = 5000`) for stability on jumper wiring; do not increase without hardware verification.

## Debugging tips
- Use `idf.py monitor` to view `ESP_LOGI/E` output; `TAG` in `VXL.c` is `VXL_FINAL`.
- If the device resets or seems stuck, check `esp_task_wdt` behavior in `task_control` (wdt configured and reset in loop). Long blocking calls must be avoided or feed the WDT.
- To verify SD logging: after boot, check `/sdcard/log.csv` (header created if absent). Failure to mount sets `sys.sd_mounted=false`.
- For hardware issues: confirm I2C addresses (LCD 0x27, SHT35 0x44) and OneWire pull-up (4.7k recommended).

## Project-specific patterns & conventions
- Buttons are active-low and debounced manually (200ms window in `task_ui`) — follow this pattern for added buttons.
- Shared state lives in `static SharedData_t sys` and is protected by a FreeRTOS mutex (`mutex`). Always lock via `xSemaphoreTake` / `xSemaphoreGive` for multi-task reads/writes.
- Time values use TickCount * `portTICK_PERIOD_MS` frequently; logging interval is `LOG_INTERVAL_MS`.
- PWM uses LEDC 10-bit (`PWM_RES` => 0..1023). Soft-start increments by steps of 10 every `SOFT_START_STEP_MS`.
- For critical, timing-sensitive bitbanged drivers (OneWire) a `portMUX_TYPE` spinlock is used (`portENTER_CRITICAL`/`portEXIT_CRITICAL`). Use this same approach if manipulating same GPIO in ISRs or different tasks.
- State machine is explicit via `SystemState_t` and transitions are performed in `task_control`. Prefer small, testable state changes rather than ad-hoc pin toggles.

## Where to look for examples (useful code excerpts)
- Main logic & patterns: `main/VXL.c` — pin definitions, drivers (I2C/SD/OneWire), tasks.
- Build config: `CMakeLists.txt`, `sdkconfig` (checked in for reproducibility).
- Hardware wiring and rationale: `README.md` and `HUONG_DAN_LAP_DAT.txt`.

## Safe modifications checklist
- Changing pins: Update macros in `main/VXL.c` and validate BOOT strapping pins for ESP32 (avoid pins that affect boot mode like GPIO0/GPIO2/GPIO12 unless intentional).
- Adding sensors: Reuse I2C bus (`i2c_init_bus()`) and adhere to same read patterns (non-blocking where possible). Add checks so `sys.env_valid`/`sys.water_valid` remain accurate.
- Changing logging cadence: update `LOG_INTERVAL_MS` and ensure `log_to_sd()` returns early if SD not mounted.

## Tests & validation examples
- Manual: Build + flash, open monitor, observe `RUNNING` log and LCD updates. Press MODE to toggle edit mode and change target humidity, verify UI behavior and that `sys.target_hum` changes.
- SD check: Remove SD, boot device and ensure mount fails gracefully (no crash), then insert SD and confirm `log.csv` header creation and periodic entries.
- Overheat path: Simulate high water temp (or set `TEMP_WATER_CRITICAL` lower temporarily) and verify transition to `STATE_PROTECT_ERR` and full case fan behavior.

## Notes for future agents
- Preserve the single-file, C-style modular structure (drivers + tasks in `VXL.c`) unless refactoring into components with clear tests.
- When adding components, add small, self-contained initialization functions (`xxx_init`) and unit-testable helper functions where possible.
- If adding long-running operations, either run them in separate tasks or ensure they yield to allow WDT resets.

---
If anything here is unclear or you'd like more examples (e.g., a sample PR to add a new I2C sensor), tell me which section to expand and I will iterate. ✅