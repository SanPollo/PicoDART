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
 * TraceUart.cpp
 *
 * TX-only software UART for diagnostic output on GPIO TRACE_TX_PIN.
 *
 * Bit format: 8 data bits, no parity, 1 stop bit, idle high.
 * Timing is provided by busy_wait_us(), which is accurate enough at 9600 baud
 * on the Pico's 125 MHz system clock (bit period = 104 µs; error < 0.2%).
 *
 * All public functions contain 'if constexpr (!TRACE_UART) return;' so that
 * the compiler eliminates them — and txByte() — entirely when TRACE_UART is
 * false.  No runtime overhead is incurred in production builds.
 *
 * TIMING NOTE:
 * When called from Core 1's bus handler (e.g. from CPS8256::out()), trace
 * output runs while /WAIT is asserted.  /WAIT freezes the PCW's Z80 and all
 * on-bus peripherals, including any software timers running on the PCW, so
 * the PCW cannot time out during the stall and resumes correctly afterwards.
 * At 9600 baud a typical 20-character message takes roughly 20 ms.  This is
 * entirely acceptable for a diagnostic build.
 *
 */

#include "TraceUart.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Bit period in microseconds: 1 000 000 / TRACE_BAUD.
// At 9600 baud this is 104 µs (rounds down from 104.17 µs; error < 0.2%).
static constexpr uint32_t BIT_US = 1000000u / TRACE_BAUD;

// ---------------------------------------------------------------------------
// Internal helper — transmit one byte, LSB first, 8N1.
// [[maybe_unused]] suppresses the -Wunused-function warning that would fire
// when TRACE_UART is false and the if constexpr guards in the public
// functions cause all call sites to be discarded by the compiler.
// ---------------------------------------------------------------------------
[[maybe_unused]] static void txByte(uint8_t b)
{
    // Start bit — drive line low.
    gpio_put(TRACE_TX_PIN, false);
    busy_wait_us(BIT_US);

    // 8 data bits, LSB first.
    for (int i = 0; i < 8; ++i)
    {
        gpio_put(TRACE_TX_PIN, (b >> i) & 1u);
        busy_wait_us(BIT_US);
    }

    // Stop bit — return line high.
    gpio_put(TRACE_TX_PIN, true);
    busy_wait_us(BIT_US);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void TraceUart::init()
{
    if constexpr (!TRACE_UART) return;

    // Set the output level BEFORE enabling the output driver so the line
    // never glitches low during initialisation.
    gpio_init(TRACE_TX_PIN);
    gpio_put(TRACE_TX_PIN, true);
    gpio_set_dir(TRACE_TX_PIN, GPIO_OUT);

    // Brief settling time before the first character.
    busy_wait_us(10000);   // 10 ms

    print("\r\n--- PerryFi TRACE_UART ---\r\n");
}

void TraceUart::putChar(char c)
{
    if constexpr (!TRACE_UART) return;
    txByte(static_cast<uint8_t>(c));
}

void TraceUart::print(const char *s)
{
    if constexpr (!TRACE_UART) return;
    while (*s)
        txByte(static_cast<uint8_t>(*s++));
}

void TraceUart::printHex(uint32_t value, int digits)
{
    if constexpr (!TRACE_UART) return;
    if (digits < 1) digits = 1;
    if (digits > 8) digits = 8;
    for (int shift = (digits - 1) * 4; shift >= 0; shift -= 4)
    {
        uint8_t nibble = (value >> shift) & 0x0Fu;
        txByte(static_cast<uint8_t>(nibble < 10u ? ('0' + nibble)
                                                  : ('A' + nibble - 10u)));
    }
}

void TraceUart::printDec(uint32_t value)
{
    if constexpr (!TRACE_UART) return;

    if (value == 0u)
    {
        txByte('0');
        return;
    }

    // Collect digits in reverse order (most-significant last).
    char buf[10];   // 2^32 = 4 294 967 296 — at most 10 decimal digits
    int  pos = 0;
    while (value > 0u && pos < 10)
    {
        buf[pos++] = static_cast<char>('0' + (value % 10u));
        value /= 10u;
    }

    // Emit digits most-significant first.
    for (int i = pos - 1; i >= 0; --i)
        txByte(static_cast<uint8_t>(buf[i]));
}

void TraceUart::println()
{
    if constexpr (!TRACE_UART) return;
    txByte('\r');
    txByte('\n');
}