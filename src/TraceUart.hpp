/*
 * PicoDART - Amstrad CPS8256 emulator for the Raspberry Pi Pico
 *
 * Copyright 2026 Nick J. Date
 *
 * PicoDART is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * PicoDART is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along
 * with PicoDART. If not, see: https://www.gnu.org/licenses/
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/*
 * TraceUart.hpp
 *
 * Lightweight TX-only software UART for diagnostic output on GPIO 27.
 *
 * All functions are guarded by 'if constexpr (TRACE_UART)' internally,
 * so they compile to nothing when TRACE_UART == false.  Callers do not
 * need their own guards.
 *
 */

#pragma once

#include <cstdint>
#include "config.hpp"

namespace TraceUart
{
    // Initialise GPIO TRACE_TX_PIN as an idle-high output and emit a startup
    // banner so the receiver knows the trace port is alive before any
    // instrumentation fires.  Call once from main() before the PicoComms
    // constructor and before Core 1 is launched.
    void init();

    // Transmit a single character.
    void putChar(char c);

    // Transmit a null-terminated string.
    void print(const char *s);

    // Transmit exactly 'digits' uppercase hex digits for the low bits of
    // 'value'.  'digits' is clamped to 1–8; leading zeros are always printed.
    void printHex(uint32_t value, int digits = 2);

    // Transmit a decimal representation of 'value' (no leading zeros; always
    // at least one digit).
    void printDec(uint32_t value);

    // Transmit CR LF.
    void println();
}