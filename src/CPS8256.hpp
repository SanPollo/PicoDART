/*
 * PicoDART - Amstrad CPS8256 emulator for the Raspberry Pi Pico
 *
 * Copyright 1996, 2001 John Elliott <seasip.webmaster@gmail.com>
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
 * CPS8256.hpp
 *
 * Emulation of the Amstrad CPS8256 serial interface card,
 * adapted from the JOYCE PCW emulator's JoyceCPS.hxx for use on the
 * Raspberry Pi Pico.
 *
 * The CPS8256 contains:
 *   - A Z80 DART (two serial channels)
 *   - An 8253 Programmable Interval Timer (used solely for baud-rate
 *     generation)
 *   - A Centronics parallel port latch
 *
 * For PerryFi, Channel A of the DART carries the serial (modem) traffic.
 * Channel B and the Centronics latch are emulated at register level — no
 * physical printer exists, but software addressing them sees a real,
 * always-ready device (see the Centronics notes further down).  The 8253
 * baud-rate arithmetic is reproduced in software; no 8253 hardware is
 * needed.
 *
 * I/O port map (low nibble of Z80 port address):
 *   0  DART Channel A data
 *   1  DART Channel A control
 *   2  DART Channel B data     (Centronics printer channel)
 *   3  DART Channel B control  (Centronics printer channel)
 *   4  8253 timer data (TX baud divisor)
 *   5  8253 timer data (RX baud divisor)
 *   7  8253 timer control word
 *   8  Centronics character latch (committed when Channel B drops STROBE)
 *
 * Original source:
 *   JOYCE v1.90 - Amstrad PCW emulator
 *   Copyright (C) 1996, 2001  John Elliott <seasip.webmaster@gmail.com>
 *   https://www.seasip.info/Unix/Joyce/
 *
 * Modifications for Pico:
 *   - Renamed JoyceCPS → CPS8256
 *   - All JOYCE base classes, UI, SDL, XML removed
 *   - Z80DartB (printer channel) retained, wired to a virtual always-ready
 *     printer (isBusy()/dropStrobe() below); the port-8 Centronics latch
 *     is retained with it
 *   - tick() replaced by poll(), which returns a bool rather than
 *     calling into a JOYCE CPU object
 *
 */

#pragma once

#include <cstdint>
#include "PicoComms.hpp"
#include "Z80Dart.hpp"

// ---------------------------------------------------------------------------
// CPS8256 — top-level CPS8256 card emulation
// ---------------------------------------------------------------------------
class CPS8256
{
public:
    // Construct with a pre-initialised PicoComms instance.
    // PicoComms must outlive this object.
    explicit CPS8256(PcwComms *comms);

    // Reset the card to its power-on state.
    void reset();

    // ---------------------------------------------------------------------------
    // Bus interface — called by the bus handler for every Z80 I/O cycle
    // that the address decoder has determined belongs to this card.
    //
    // 'port' is the raw Z80 port address; only the low 4 bits are used,
    // matching the CPS8256 hardware decoding.
    // ---------------------------------------------------------------------------
    uint8_t in (uint8_t port);
    void    out(uint8_t port, uint8_t value);

    // ---------------------------------------------------------------------------
    // poll() — call this regularly from the main loop (or a timer ISR).
    //
    // Returns true if the DART Channel A is configured for interrupt-driven
    // mode AND a received character is waiting.  When this returns true the
    // caller should assert the /INT line to the PCW's Z80.
    //
    // In polled mode (the default for most CP/M comms software) this will
    // always return false and can safely be ignored.
    // ---------------------------------------------------------------------------
    bool poll();

    // ---------------------------------------------------------------------------
    // Centronics printer interface — called by Z80DartB (Channel B).
    //
    // PerryFi has no physical Centronics port, so we present the benign
    // "printer connected and ready" state and discard printed characters.
    // The register-level behaviour (STROBE edge detection, the port-8 data
    // latch, CTS status) is fully emulated so software addressing Channel B
    // sees a real device rather than a floating bus.
    // ---------------------------------------------------------------------------
    bool isBusy() const { return false; }   // "printer ready" (BUSY deasserted)
    void dropStrobe()   { /* discard m_cench — no physical printer */ }

private:
    PcwComms *m_comms;
    Z80DartA  m_dartA;
    Z80DartB  m_dartB;
    int       m_cench;   // Character in the Centronics I/O latch (port 8)

    // 8253 baud-rate state machine.
    // m_baudmode tracks which phase of the three-byte programming sequence
    // the 8253 is in:
    //   0 = idle (waiting for a control word)
    //   1 = expecting TX divisor low byte
    //   2 = expecting TX divisor high byte
    //   3 = expecting RX divisor low byte
    //   4 = expecting RX divisor high byte
    int m_baudmode;
    int m_txbaud;
    int m_rxbaud;
    int m_baudbuf;   // Holds the low byte between the two divisor writes

    // Counter used by poll() to avoid checking on every call.
    int m_pollTick;
};