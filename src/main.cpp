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
 * main.cpp - Top-level entry point for PicoDART
 *
 * Initialisation order:
 *   1. PicoComms (initialises UART0 to the ESP8266)
 *   2. CPS8256 (initialises the emulated DART and 8253)
 *   3. BusInterface (configures GPIOs and launches Core 1)
 *   4. Main loop on Core 0: calls CPS8256::poll() and asserts /INT if needed
 *
 * Note: SDK UART stdio is intentionally NOT initialised — see the comment
 * at the top of main() for why.
 *
 */

#include "pico/stdlib.h"
#include "PicoComms.hpp"
#include "CPS8256.hpp"
#include "BusInterface.hpp"
#include "TraceUart.hpp"

int main()
{
    // Initialise the diagnostic soft-UART first so that all subsequent
    // initialisations (PicoComms constructor, Core 1 launch) can be traced.
    // TraceUart::init() is a no-op when TRACE_UART == false.
    if constexpr (TRACE_UART) {
        TraceUart::init();
    }

    // NOTE: stdio_init_all() is deliberately not called. With
    // pico_enable_stdio_uart active it would claim UART0 on GP0/GP1 for a
    // console — the very pins used here for the ESP8266 link — and would
    // hijack that link. UART stdio must never be enabled.

    // Initialise the serial comms layer (UART0 on GP0/GP1 → ESP8266).
    PicoComms comms(uart0, 9600);

    // Initialise the CPS8256 emulation layer.
    CPS8256 cps(&comms);

    // Initialise the bus interface and launch the Core 1 polling loop.
    BusInterface bus(cps);
    bus.init();

    // ---------------------------------------------------------------------------
    // Core 0 main loop.
    //
    // Core 1 handles all time-critical bus cycles autonomously.
    // Core 0 is free to handle lower-priority tasks:
    //   - Polling the CPS8256 emulation to detect pending interrupts
    //   - Any future status LED, configuration, or housekeeping tasks
    // ---------------------------------------------------------------------------
    while (true)
    {
        // Clear a BREAK condition once its duration has elapsed. sendBreak()
        // is called from Core 1 with /WAIT asserted, so it cannot block there.
        comms.serviceBreak();

        // Check whether the DART is in interrupt mode and a character is
        // waiting.  If so, assert /INT to the PCW's Z80.
        if (cps.poll())
        {
            bus.assertInt(true);

            // The Z80 will respond with an INTACK cycle, which Core 1 will
            // detect and service via the normal bus loop.  We deassert /INT
            // after a brief interval; this could later be made
            // edge-triggered by monitoring the INTACK cycle explicitly.
            sleep_us(10);
            bus.assertInt(false);
        }
    }

    return 0;
}