cmake_minimum_required(VERSION 3.13)

# PicoDART - Amstrad CPS8256 emulator for the Raspberry Pi Pico
#
# Copyright 2026 Nick J. Date
#
# PicoDART is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# PicoDART is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along
# with PicoDART. If not, see: https://www.gnu.org/licenses/
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ---------------------------------------------------------------------------
# Pull in the Pico SDK.
# The PICO_SDK_PATH environment variable must be set before running cmake,
# or you can pass it on the command line:
#   cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
# ---------------------------------------------------------------------------
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# ---------------------------------------------------------------------------
# Project definition.
# We are targeting the original Raspberry Pi Pico (RP2040).
# Change PICO_BOARD to pico_w if you switch to the Pico W in future.
# ---------------------------------------------------------------------------
project(perryfi
    VERSION     1.0
    DESCRIPTION "PicoDART - Amstrad CPS8256 emulator for the Raspberry Pi Pico"
    LANGUAGES   C CXX ASM
)

set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the Pico SDK.  Must be called after project().
pico_sdk_init()

# ---------------------------------------------------------------------------
# Main firmware executable
# ---------------------------------------------------------------------------
add_executable(perryfi
    main.cpp
    PicoComms.cpp
    Z80Dart.cpp
    CPS8256.cpp
    BusInterface.cpp
    TraceUart.cpp
)

# Headers are in the same directory as the sources.
target_include_directories(perryfi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ---------------------------------------------------------------------------
# Pico SDK libraries
#
#   pico_stdlib        — gpio, uart, time, stdio and other common functions
#   pico_multicore     — multicore_launch_core1() and related primitives
#   pico_sync          — critical_section (cross-core UART register lock)
#   hardware_uart      — low-level UART register access (uart_get_hw() etc.)
#   hardware_gpio      — low-level GPIO (gpio_get_all(), gpio_put_masked() etc.)
# ---------------------------------------------------------------------------
target_link_libraries(perryfi
    pico_stdlib
    pico_multicore
    pico_sync
    hardware_uart
    hardware_gpio
)

# ---------------------------------------------------------------------------
# stdio routing
#
# UART stdio is DISABLED: UART0's default pins are GP0/GP1, which on this
# board are D0/D1 of the PCW data bus. Routing printf there would corrupt
# the bus. Do not re-enable it.
#
# This explicit disable is required because the SDK enables UART stdio by
# default (PICO_STDIO_UART defaults to 1). USB stdio is left at its SDK
# default (PICO_STDIO_USB defaults to 0), so it is not linked in.
# ---------------------------------------------------------------------------
pico_enable_stdio_uart(perryfi 0)

# ---------------------------------------------------------------------------
# Extra outputs
#
# Generates the following alongside the ELF:
#   .uf2  — drag-and-drop firmware image for the Pico's USB bootloader
#   .hex  — Intel HEX format (useful for some programmers)
#   .bin  — raw binary
#   .map  — linker map (useful for verifying __not_in_flash_func placement)
#   .dis  — disassembly
# ---------------------------------------------------------------------------
pico_add_extra_outputs(perryfi)

# ---------------------------------------------------------------------------
# Compiler warnings
# ---------------------------------------------------------------------------
target_compile_options(perryfi PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter   # PcwComms interface stubs have intentional no-ops
)