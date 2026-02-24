# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Important

The `esp-idf-dev` skill is available — use it when writing or modifying ESP-IDF code. It has up-to-date knowledge of ESP-IDF v5.4/5.5 APIs, FreeRTOS patterns, peripheral drivers, and the CMake build system.

Use web search to validate ESP-IDF API usage, function signatures, and component configurations when not fully confident. ESP-IDF evolves across versions and training data may be outdated — always verify against current docs rather than guessing.

## Repository Purpose

Monorepo for prototyping ESP-IDF firmware projects. Each subdirectory is an independent ESP-IDF project targeting a specific board/peripheral combination. Projects here are experimental — once mature, they get moved to their own repositories.

## Build Commands

All commands run from within a sub-project directory (e.g., `neopixel-hello/`):

```bash
idf.py set-target <chip>    # e.g., esp32c6 — run once per project
idf.py build                # build firmware
idf.py flash                # flash to connected board
idf.py monitor              # serial monitor (Ctrl+] to exit)
idf.py flash monitor        # flash then immediately monitor
idf.py menuconfig           # interactive sdkconfig editor
idf.py fullclean            # wipe build dir for clean rebuild
```

Component manager commands (dependencies declared in `main/idf_component.yml`):
```bash
idf.py add-dependency "espressif/led_strip^3.0.0"   # add a component
idf.py update-dependencies                            # refresh managed_components/
```

## Project Template

Each sub-project follows the standard ESP-IDF structure:

```
project-name/
├── CMakeLists.txt              # Root: cmake_minimum_required + include(project.cmake) + project()
├── main/
│   ├── CMakeLists.txt          # idf_component_register(SRCS, INCLUDE_DIRS, REQUIRES)
│   ├── idf_component.yml       # External component dependencies (espressif registry)
│   └── main.c                  # Application entry point (app_main)
├── sdkconfig.defaults          # Minimal config overrides (committed)
├── sdkconfig                   # Full generated config (gitignored)
├── dependencies.lock           # Locked component versions
└── managed_components/         # Auto-downloaded components (gitignored)
```

## Architecture Notes

- **ESP-IDF v5.4.1** with the component manager (manifest v2.0.0)
- **FreeRTOS task model**: each functional concern gets its own task with explicit priority, stack size, and core affinity where needed
- **No Arduino abstractions**: uses ESP-IDF APIs directly (gpio, rmt, i2c drivers, esp_timer, etc.)
- **ESP32-C6 specifics**: uses USB Serial/JTAG driver (`usb_serial_jtag_driver_install`) instead of UART0 for console I/O
- Component `REQUIRES` declarations in `main/CMakeLists.txt` must explicitly list all ESP driver components used (e.g., `esp_driver_gpio`, `esp_driver_rmt`)

## Conventions

- Pin definitions and hardware config as `#define` constants at top of source files
- `volatile` variables for cross-task shared state (simple cases); queues/semaphores for complex synchronization
- Non-blocking timing patterns (`esp_timer_get_time()` or `vTaskDelay`-based loops, never busy-wait)
- WiFi credentials and API keys go in a gitignored `secrets.h` — provide a `secrets.h.example` template
