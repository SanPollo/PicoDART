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
 */
 
/*
 * config.hpp - Central hardware configuration
 *
 * All GPIO pin assignments are defined here.  This is the only file that
 * needs to be edited when the PCB layout changes.
 *
 * Derived values (masks, counts) that follow mathematically from the pin
 * assignments are computed in BusInterface.hpp and should not need manual
 * adjustment as long as the constraints noted below are respected.
 *
 * ---------------------------------------------------------------------------
 * Constraints
 * ---------------------------------------------------------------------------
 *
 *  1. BUS_DATA_BASE_PIN must be the lowest-numbered of the eight data bus
 *     pins, and D0–D7 must occupy eight consecutive GPIOs from that base.
 *     This allows the entire data byte to be read or written with a single
 *     masked GPIO operation.
 *
 *  2. BUS_ADDR_BASE_PIN must be the lowest-numbered of the four address
 *     pins, and A0–A3 must occupy four consecutive GPIOs from that base,
 *     for the same reason.
 *
 *  3. UART0 TX/RX pins must be a valid UART0 pin pair for the RP2040.
 *     Valid UART0 TX pins: GP0, GP12, GP16, GP28.
 *     Valid UART0 RX pins: GP1, GP13, GP17, GP29.
 *     (The RP2040 hardware UART pad mapping is fixed: GP0/GP1 can only be
 *     UART0, never UART1 — hence uart0 is passed to PicoComms in main.cpp.)
 *
 *  4. No two signals may share the same GPIO number.
 *
 *  5. All signals from the 5 V PCW bus must be level-shifted before
 *     reaching the Pico.  The RP2040 GPIOs are not 5 V tolerant.
 *     See BusInterface.hpp for level-shifting guidance.
 *
 * ---------------------------------------------------------------------------
 * Pin allocation
 * ---------------------------------------------------------------------------
 *
 *  GP0         : UART0 TX (→ ESP8266 RX)
 *  GP1         : UART0 RX (← ESP8266 TX)
 *  GP2 – GP9   : D0–D7   (data bus, via 74LVC245)
 *  GP10        : BUS_DIR  (74LVC245 direction control)
 *  GP11        : /CS      (chip select from address decoder, active low)
 *  GP12        : /IORQ    (Z80 I/O request, active low)
 *  GP13        : /RD      (Z80 read strobe, active low)
 *  GP14        : /WR      (Z80 write strobe, active low)
 *  GP15        : /WAIT    (Z80 wait request, active low, output)
 *  GP16        : /INT     (Z80 interrupt request, active low, output)
 *  GP17 – GP20 : A0–A3   (address bus low nibble, inputs)
 *  GP21        : CTSA     (← ESP8266 CTS, active low, input)
 *  GP22        : DTRA     (→ ESP8266 DTR, active low, output)
 *  GP26        : /INT_OE  (74HCT125 gate 2 output-enable, active low, output)
 *                Requires a 10k pull-up to VCC_3V3 on the PCB.
 *
 */

#pragma once

#include <cstdint>
#include "pico/stdlib.h"

// ---------------------------------------------------------------------------
// Data bus (D0–D7)
// Must be eight consecutive GPIOs.  Change BUS_DATA_BASE_PIN to relocate
// the entire block; the mask is computed automatically in BusInterface.hpp.
// ---------------------------------------------------------------------------
static constexpr uint BUS_DATA_BASE_PIN  =  2;   // D0 — D7 follow on GP3–GP9

// ---------------------------------------------------------------------------
// 74LVC245 bidirectional buffer direction control (U4)
//
// U4's A side is wired to the 5 V PCW data bus; its B side to GP2–GP9.
// Per the 74LVC245 function table (with OE low):
//   DIR = 1 (HIGH) → A→B : PCW drives the Pico   (idle default, write cycles)
//   DIR = 0 (LOW)  → B→A : Pico drives the PCW   (read cycles only)
//
// NOTE: these two states are easy to get the wrong way round. Driving them
// inverted drives the level shifter against the Z80 on every selected cycle
// and leaves the PCW bus floating during reads — the CPS8256 could never be
// detected. Do not "simplify" this.
// ---------------------------------------------------------------------------
static constexpr uint BUS_DIR_PIN        = 10;

// ---------------------------------------------------------------------------
// Control inputs from the PCW Z80 bus (all active low)
// ---------------------------------------------------------------------------
static constexpr uint BUS_CS_PIN         = 11;   // /CS   chip select
static constexpr uint BUS_IORQ_PIN       = 12;   // /IORQ I/O request
static constexpr uint BUS_RD_PIN         = 13;   // /RD   read strobe
static constexpr uint BUS_WR_PIN         = 14;   // /WR   write strobe

// ---------------------------------------------------------------------------
// Outputs to the PCW Z80 bus (both active low)
// ---------------------------------------------------------------------------
static constexpr uint BUS_WAIT_PIN       = 15;   // /WAIT
static constexpr uint BUS_INT_PIN        = 16;   // /INT

// ---------------------------------------------------------------------------
// /INT output-enable control (74HCT125 gate 2, pin 4 / 2OE)
//
// /INT is a genuinely shared PCW system line — the FDC and the system timer
// can also drive it — so this gate must behave like an open-drain output:
// tri-stated (OE high / disabled) whenever we are not actively asserting, and
// only enabled at the same instant BUS_INT_PIN is already low. See
// assertInt() for the required ordering.
//
// A 10k pull-up to VCC_3V3 on this net keeps the gate disabled by default
// at power-on/reset, before this code has run.
// ---------------------------------------------------------------------------
static constexpr uint BUS_INT_OE_PIN     = 26;   // GPIO26 → U2 pin 4 (2OE)

// ---------------------------------------------------------------------------
// Address bus low nibble (A0–A3, inputs)
// Must be four consecutive GPIOs.  Change BUS_ADDR_BASE_PIN to relocate
// the entire block; the mask is computed automatically in BusInterface.hpp.
// ---------------------------------------------------------------------------
static constexpr uint BUS_ADDR_BASE_PIN  = 17;   // A0 — A3 follow on GP18–GP20

// ---------------------------------------------------------------------------
// UART0 — serial link to ESP8266 (ESP-12F)
// Must be a valid UART0 TX/RX pair for the RP2040 (see constraint 3 above).
// ---------------------------------------------------------------------------
static constexpr uint UART_TX_PIN        =  0;   // GP0 → ESP8266 RX
static constexpr uint UART_RX_PIN        =  1;   // GP1 ← ESP8266 TX

// ---------------------------------------------------------------------------
// ESP8266 handshaking lines.
// Both signals are 3.3 V and connect directly to the ESP-12F with
// no level shifting required.
// Both signals are active low, matching standard RS-232 convention.
// ---------------------------------------------------------------------------
static constexpr uint ESP8266_CTS_PIN    = 21;   // GP21 ← ESP8266 CTS (CTSA)
static constexpr uint ESP8266_DTR_PIN    = 22;   // GP22 → ESP8266 DTR (DTRA)

// ---------------------------------------------------------------------------
// TRACE_UART — TX-only software-UART diagnostic output on GPIO 27.
//
// When true, TraceUart::init() configures GPIO TRACE_TX_PIN as an idle-high
// output and subsequent calls from CPS8256.cpp and PicoComms.cpp emit messages
// that trace the 8253 baud-rate programming sequence and every UART
// reconfiguration.  Connect a USB-to-TTL adapter (RX to GP27, GND to GND)
// and open a terminal at TRACE_BAUD to observe the output.
//
// When false (default), all TraceUart calls compile to nothing; the
// production firmware is completely unaffected.
//
// WARNING: trace output called from Core 1 (the bus handler) runs while
// /WAIT is asserted.  /WAIT freezes the PCW's Z80 and all on-bus peripherals,
// so the PCW cannot time out during the stall and resumes correctly
// afterwards.  This is acceptable for a diagnostic build; never enable in
// production.
// ---------------------------------------------------------------------------
static constexpr bool TRACE_UART   = false;
static constexpr uint TRACE_TX_PIN = 27;     // GP27 → USB-to-TTL RX
static constexpr uint TRACE_BAUD   = 9600;