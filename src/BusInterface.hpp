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
 * BusInterface.hpp - Z80 I/O bus interface implementation
 *
 * This module runs on Core 1 of the RP2040 and watches the PCW's Z80
 * expansion bus for I/O cycles addressed to the CPS8256.  When it detects
 * a cycle it asserts /WAIT to freeze the Z80, services the read or write
 * via the CPS8256 emulation layer, then releases /WAIT.
 *
 * ---------------------------------------------------------------------------
 * Timing rationale
 * ---------------------------------------------------------------------------
 * The PCW's Z80 runs at approximately 4 MHz.  An I/O cycle is:
 *
 *   T1 (/IORQ asserted) → TW (wait states) → T2 → T3
 *
 * The Z80 samples /WAIT at the falling edge of the T2 clock, approximately
 * 500 ns after /IORQ goes low.  A GPIO interrupt handler running from flash
 * can take 500 ns–6 µs depending on cache state — far too variable.
 *
 * Instead, Core 1 runs a dedicated tight polling loop from SRAM
 * (__not_in_flash_func).  At 125 MHz each loop iteration takes ~5 cycles
 * (40 ns), giving a worst-case detection latency of roughly 80 ns — well
 * within the Z80's window even before any wait states are inserted.
 *
 * SRAM placement of the bus loop has been verified using arm-none-eabi-nm;
 * the symbol _ZN12BusInterface7busLoopEv resolves to the SRAM range
 * (0x20000000–0x2003FFFF), confirming that consistent timing is achieved
 * without any additional external hardware.
 *
 * ---------------------------------------------------------------------------
 * GPIO pin allocation
 * ---------------------------------------------------------------------------
 * All pin assignments are defined in config.hpp.  Refer to that file to
 * change the allocation to suit a different PCB layout.
 *
 * ---------------------------------------------------------------------------
 * Level shifting
 * ---------------------------------------------------------------------------
 * The RP2040 GPIOs are NOT 5 V tolerant (maximum 3.63 V).  All signals
 * from the 5 V PCW bus MUST be level-shifted before reaching the Pico:
 *
 *  Data bus (D0–D7):       74LVC245 bidirectional buffer.
 *                           OE gated by /CS (active low).
 *                           DIR driven by BUS_DIR_PIN.
 *                           Pico GPIOs switched between input and output
 *                           to match the 74LVC245 direction.
 *
 *  Control/address inputs  One or two 74LVC244 octal buffers (input only),
 *  (/CS, /IORQ, /RD, /WR,  translating 5 V → 3.3 V.
 *   A0–A3):
 *
 *  /WAIT (output):          Buffered through a 74HCT125 gate. OE for this
 *                           gate is hardware-gated by /CS, so it only drives
 *                           the bus during a cycle addressed to this device
 *                           — no firmware control needed.
 *
 *  /INT (output):           Buffered through a second 74HCT125 gate. Unlike
 *                           /WAIT, /INT is shared with other PCW peripherals
 *                           (the FDC and system timer can also drive it), so
 *                           gating on /CS alone would be wrong — an
 *                           interrupt must be assertable even when this
 *                           device isn't currently addressed. Its OE is
 *                           instead firmware-controlled via BUS_INT_OE_PIN;
 *                           see assertInt() for the required ordering.
 *
 */

#pragma once

#include <cstdint>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "config.hpp"
#include "CPS8256.hpp"

// ---------------------------------------------------------------------------
// Derived masks — computed from the pin assignments in config.hpp.
// These do not need to be edited manually.
// ---------------------------------------------------------------------------

static constexpr uint     BUS_DATA_PIN_COUNT =  8;
static constexpr uint32_t BUS_DATA_MASK  = 0xFFu << BUS_DATA_BASE_PIN;

static constexpr uint     BUS_ADDR_PIN_COUNT =  4;
static constexpr uint32_t BUS_ADDR_MASK  = 0x0Fu << BUS_ADDR_BASE_PIN;

static constexpr uint32_t BUS_CS_MASK   = 1u << BUS_CS_PIN;
static constexpr uint32_t BUS_IORQ_MASK = 1u << BUS_IORQ_PIN;
static constexpr uint32_t BUS_RD_MASK   = 1u << BUS_RD_PIN;
static constexpr uint32_t BUS_WR_MASK   = 1u << BUS_WR_PIN;

// ---------------------------------------------------------------------------
// BusInterface
// ---------------------------------------------------------------------------
class BusInterface
{
public:
    // Construct with a reference to the CPS8256 emulation object.
    // The CPS8256 instance must outlive the BusInterface.
    explicit BusInterface(CPS8256 &cps);

    // Initialise all GPIOs and launch the bus-polling loop on Core 1.
    // Call once from main() after stdio and PicoComms have been initialised.
    void init();

    // Assert or release the /INT line.
    // Call from Core 0 when CPS8256::poll() returns true.
    void assertInt(bool active);

private:
    CPS8256 &m_cps;

    // GPIO setup helpers
    void setupInputPin (uint pin);
    void setupOutputPin(uint pin, bool initialState);

    // Bus cycle handlers — called from Core 1 only
    void handleRead (uint8_t port);
    void handleWrite(uint8_t port);

    // The Core 1 entry point (static so it can be passed to
    // multicore_launch_core1; uses a global pointer to reach the instance).
    static void core1Entry();
    void busLoop();   // The actual tight polling loop
};