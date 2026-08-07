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
 * BusInterface.cpp - Z80 I/O bus interface implementation
 *
 */

#include "BusInterface.hpp"
#include "pico/multicore.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// Global pointer used by the static Core 1 entry point to reach the
// BusInterface instance.  There is only ever one BusInterface, so a global
// is appropriate here.
// ---------------------------------------------------------------------------
static BusInterface *g_busInterface = nullptr;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

BusInterface::BusInterface(CPS8256 &cps)
    : m_cps(cps)
{
    g_busInterface = this;
}

// ---------------------------------------------------------------------------
// init() — configure GPIOs and launch Core 1
// ---------------------------------------------------------------------------

void BusInterface::init()
{
    // --- Data bus (GP2–GP9): start as inputs (Z80 drives us by default) ---
    for (uint pin = BUS_DATA_BASE_PIN;
         pin < BUS_DATA_BASE_PIN + BUS_DATA_PIN_COUNT; ++pin)
    {
        setupInputPin(pin);
    }

    // --- 74LVC245 direction pin: output, default A→B (Z80 drives Pico) ---
    // On the real 74LVC245, A→B requires DIR HIGH (see config.hpp).
    setupOutputPin(BUS_DIR_PIN, true);

    // --- Control inputs ---
    setupInputPin(BUS_CS_PIN);
    setupInputPin(BUS_IORQ_PIN);
    setupInputPin(BUS_RD_PIN);
    setupInputPin(BUS_WR_PIN);

    // --- /WAIT: output, deasserted (high) initially ---
    // Driven as open-drain: we pull low to assert, release (input) to deassert.
    // For simplicity we drive it as a push-pull output here; if the PCW bus
    // has other /WAIT sources, change to open-drain with gpio_set_pulls().
    setupOutputPin(BUS_WAIT_PIN, true);   // high = deasserted

    // --- /INT: output, deasserted (high) initially ---
    setupOutputPin(BUS_INT_PIN, true);    // high = deasserted

    // --- /INT output-enable: output, DISABLED initially ---
    // Must be set up as disabled (high) before assertInt() can ever be
    // called. A physical 10k pull-up to VCC_3V3 backs this up during the
    // brief window between power-on and this line executing.
    setupOutputPin(BUS_INT_OE_PIN, true); // high = gate disabled (tri-state)

    // --- Address lines (GP17–GP20): inputs ---
    for (uint pin = BUS_ADDR_BASE_PIN;
         pin < BUS_ADDR_BASE_PIN + BUS_ADDR_PIN_COUNT; ++pin)
    {
        setupInputPin(pin);
    }

    // Launch the bus polling loop on Core 1.
    multicore_launch_core1(core1Entry);
}

// ---------------------------------------------------------------------------
// assertInt() — drive /INT from Core 0
// ---------------------------------------------------------------------------

void BusInterface::assertInt(bool active)
{
    // /INT is a shared PCW system line (the FDC and system timer can also
    // drive it), so the 74HCT125 gate feeding it must behave like an
    // open-drain output: enabled (driving) only while genuinely asserting
    // low, tri-stated the rest of the time. Getting the order of these two
    // writes wrong would let the gate briefly drive the shared line high
    // while some other device is holding it low, or vice versa.
    //
    // active = true  → assert /INT (drive low):
    //   set BUS_INT_PIN low FIRST, then enable the gate — so the gate
    //   never turns on while its input is still high.
    // active = false → deassert /INT (release to high-Z):
    //   disable the gate FIRST, then release BUS_INT_PIN high — so the
    //   bus is already tri-stated before the input level changes.
    if (active)
    {
        gpio_put(BUS_INT_PIN, false);      // drive low
        gpio_put(BUS_INT_OE_PIN, false);   // then enable (active low OE)
    }
    else
    {
        gpio_put(BUS_INT_OE_PIN, true);    // disable (tri-state) first
        gpio_put(BUS_INT_PIN, true);       // then release high
    }
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

void BusInterface::setupInputPin(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

void BusInterface::setupOutputPin(uint pin, bool initialState)
{
    gpio_init(pin);
    gpio_put(pin, initialState);          // set state before enabling output
    gpio_set_dir(pin, GPIO_OUT);
}

// ---------------------------------------------------------------------------
// handleRead() — service a Z80 I/O read cycle
//
// Called from Core 1 with /WAIT already asserted.
// Reads the port number from the address lines, asks CPS8256 for the value,
// drives it onto the data bus, then releases /WAIT.
// ---------------------------------------------------------------------------

void __not_in_flash_func(BusInterface::handleRead)(uint8_t port)
{
    uint8_t value = m_cps.in(port);

    // Switch 74LVC245 to B→A (Pico drives Z80): DIR LOW on the real chip.
    // Order matters: flip DIR first (U4's B pins stop driving toward the
    // Pico), THEN turn the Pico's data pins into outputs — the other way
    // round would briefly fight U4's outputs pin-for-pin.
    gpio_put(BUS_DIR_PIN, false);

    // Load the value into the GPIO output latches BEFORE enabling the
    // output drivers.  gpio_put_masked() works regardless of pin direction,
    // so this order means the bus never momentarily shows a stale byte left
    // in the latches from a previous read.
    gpio_put_masked(BUS_DATA_MASK,
                    static_cast<uint32_t>(value) << BUS_DATA_BASE_PIN);
    gpio_set_dir_out_masked(BUS_DATA_MASK);

    // Release /WAIT — the Z80 will latch the data on the next rising clock.
    gpio_put(BUS_WAIT_PIN, true);

    // Hold the data stable until /RD is released by the Z80.
    // At 4 MHz, /RD is typically asserted for ~250 ns after /WAIT releases.
    // We busy-wait here; at 125 MHz this loop completes in ~5–10 ns per
    // iteration, so at most ~50 iterations are needed.
    while (!(gpio_get_all() & BUS_RD_MASK)) { tight_loop_contents(); }

    // Return data bus to input and 74LVC245 to A→B (Z80 drives Pico).
    // Same ordering rule in reverse: Pico pins back to input FIRST, then
    // DIR HIGH — flipping DIR while the Pico is still driving would fight
    // U4's B-side outputs.
    gpio_set_dir_in_masked(BUS_DATA_MASK);
    gpio_put(BUS_DIR_PIN, true);
}

// ---------------------------------------------------------------------------
// handleWrite() — service a Z80 I/O write cycle
//
// Called from Core 1 with /WAIT already asserted.
// Reads the data byte from the bus, passes it to CPS8256, then releases /WAIT.
// ---------------------------------------------------------------------------

void __not_in_flash_func(BusInterface::handleWrite)(uint8_t port)
{
    // Data bus is already configured as inputs (Z80 is driving).
    // 74LVC245 direction is already A→B.
    uint8_t value = static_cast<uint8_t>(
        (gpio_get_all() & BUS_DATA_MASK) >> BUS_DATA_BASE_PIN);

    m_cps.out(port, value);

    // Release /WAIT.
    gpio_put(BUS_WAIT_PIN, true);
}

// ---------------------------------------------------------------------------
// Core 1 entry point and bus polling loop
// ---------------------------------------------------------------------------

// Static trampoline — called by the Pico SDK when Core 1 starts.
void BusInterface::core1Entry()
{
    g_busInterface->busLoop();
}

// Upper bound on the wait-for-strobe spin in busLoop() below.
//
// On a genuine Z80 I/O cycle, /RD or /WR becomes visible within one or two
// loop iterations of /IORQ (the strobes change on the same clock edge as
// /IORQ; the only lag is buffer propagation plus the RP2040's per-pin input
// synchronisers).  64 iterations is roughly 3-5 µs at 125 MHz — two orders
// of magnitude of margin for catching a real strobe — while keeping the
// stall added to a spuriously-decoded INTACK cycle (see busLoop) far too
// short to matter.
static constexpr uint32_t STROBE_WAIT_LIMIT = 64;

// The tight polling loop runs from SRAM to guarantee consistent timing.
// SRAM placement has been verified — see the timing rationale in
// BusInterface.hpp for details.  __not_in_flash_func ensures the linker
// maintains this placement in all future builds.
void __not_in_flash_func(BusInterface::busLoop)()
{
    while (true)
    {
        // Read all GPIOs in a single atomic operation.
        uint32_t pins = gpio_get_all();

        // We only care when both /CS and /IORQ are low (active).
        // All other states are idle.
        if (pins & BUS_CS_MASK)   continue;
        if (pins & BUS_IORQ_MASK) continue;

        // A bus cycle is in progress.  Assert /WAIT immediately to freeze
        // the Z80 while we service the request.
        gpio_put(BUS_WAIT_PIN, false);

        // Extract the port address from A0–A3.
        uint8_t port = static_cast<uint8_t>(
            (pins & BUS_ADDR_MASK) >> BUS_ADDR_BASE_PIN);

        // Determine whether this is a read or a write.
        //
        // On a Z80 *I/O* cycle the strobe is asserted on the same clock
        // edge as /IORQ (for both reads and writes), so on a genuine cycle
        // /RD or /WR becomes visible within a few tens of nanoseconds; the
        // only reason we can arrive here with neither strobe visible yet is
        // buffer propagation plus the RP2040's per-pin input synchronisers
        // presenting the edges a sample or two apart.  We therefore keep
        // /WAIT asserted and re-read the strobes until one appears — which
        // fixes the write-dropping bug in the original release-immediately
        // else-branch.
        //
        // The wait MUST be bounded, because there is one Z80 cycle that
        // asserts /IORQ with NO strobe, ever: the interrupt-acknowledge
        // (INTACK) cycle.  During INTACK the address bus carries the PC of
        // the interrupted instruction, and this board's address decoder
        // (74LS138: enabled by /IORQ low and A4 low, selecting on
        // A7:A6:A5 = 111 — /M1 is NOT part of the decode) will therefore
        // assert /CS spuriously whenever that PC's low byte happens to fall
        // in 0xE0–0xE7 (~3% of interrupts, and the PCW's system timer
        // interrupts continuously).  The Z80 samples /WAIT during INTACK,
        // so an UNBOUNDED wait here freezes the Z80 with /IORQ held low —
        // neither exit condition can ever fire, and the machine hangs at
        // boot.
        //
        // Genuine cycles show a strobe almost immediately; if none appears
        // within STROBE_WAIT_LIMIT iterations this is an INTACK (or a
        // glitch), so release /WAIT and let the cycle finish untouched.
        // Not driving a vector is correct: the PCW runs in IM 1, in which
        // the Z80 ignores the data bus during INTACK.
        //
        // The loop deliberately does NOT exit early if /CS or /IORQ appear
        // to deassert mid-spin.  Such a single-sample check is tempting, but
        // noise on the PCW backplane (from the DRAM refresh, system timer,
        // or FDC) can glitch /CS momentarily even with /WAIT asserted and
        // the Z80 frozen.  A single-sample deassert on a genuine read cycle
        // would cause the loop to bail, release /WAIT without driving the
        // data bus, and the Z80 would then latch 0xFF from the floating
        // pull-ups.  Waiting the full STROBE_WAIT_LIMIT on any false trigger
        // or INTACK adds at most a few microseconds of extra /WAIT hold,
        // which is harmless.
        bool serviced = false;
        uint32_t deassert_count = 0;
        for (uint32_t spin = 0; spin < STROBE_WAIT_LIMIT; ++spin)
        {
            pins = gpio_get_all();

            if (!(pins & BUS_RD_MASK))        // /RD low → Z80 is reading
            {
                handleRead(port);            // releases /WAIT internally
                serviced = true;
                break;
            }
            if (!(pins & BUS_WR_MASK))        // /WR low → Z80 is writing
            {
                handleWrite(port);           // releases /WAIT internally
                serviced = true;
                break;
            }
            // If /IORQ or /CS appear to have deasserted, require three
            // consecutive samples before treating the cycle as gone.  A
            // single-sample glitch on the PCW backplane (DRAM refresh, FDC,
            // bus noise) cannot abort a genuine read cycle — that was the
            // mechanism producing 0xFF.  A genuine early deassert (false
            // trigger, or /WAIT missed by the Z80)
            // holds for at least three samples and still exits safely,
            // preserving the deadlock-prevention the original check provided.
            if ((pins & BUS_IORQ_MASK) || (pins & BUS_CS_MASK))
            {
                if (++deassert_count >= 3) { break; }
            }
            else
            {
                deassert_count = 0;
            }
            // Otherwise keep /WAIT asserted and keep watching (bounded).
        }
        if (!serviced)
        {
            // Timed out (INTACK) or the select vanished (glitch): stop
            // stalling the Z80 and let the cycle complete on its own.
            gpio_put(BUS_WAIT_PIN, true);
        }

        // Wait for /IORQ to deassert before looking for the next cycle,
        // to avoid re-triggering on the same cycle.
        while (!(gpio_get_all() & BUS_IORQ_MASK)) { tight_loop_contents(); }
    }
}