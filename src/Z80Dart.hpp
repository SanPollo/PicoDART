/*
 * PicoDART - Amstrad CPS8256 emulator for the Raspberry Pi Pico
 *
 * Copyright 1996, 2001, 2020 John Elliott <seasip.webmaster@gmail.com>
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
 * Z80Dart.hpp
 *
 * Emulation of the Z80 DART (Dual Asynchronous Receiver/Transmitter),
 * adapted from the JOYCE PCW emulator's Z80Dart.hxx for use on the
 * Raspberry Pi Pico.
 *
 * Both channels are implemented: Channel A (serial port) drives the
 * PicoComms serial layer, and Channel B (Centronics parallel printer
 * port) is emulated at register level so that software addressing it
 * sees a real, always-ready device — see the Z80DartB class notes below.
 *
 * Original source:
 *   JOYCE v1.90 - Amstrad PCW emulator
 *   Copyright (C) 1996, 2001  John Elliott <seasip.webmaster@gmail.com>
 *   https://www.seasip.info/Unix/Joyce/
 *
 * Modifications for Pico:
 *   - zbyte replaced with uint8_t
 *   - JOYCE-specific macros (XLOG, joyce_dprintf) removed
 *   - Z80DartB (printer channel) retained: it reports printer status to,
 *     and latches characters through, its owning CPS8256
 *   - No dependency on any JOYCE headers
 *
 */

#pragma once

#include <cstdint>
#include <cstring>      // memset
#include "PicoComms.hpp"

class CPS8256;   // forward declaration — Channel B reports printer status
                 // to, and latches characters through, its owning CPS8256.

// ---------------------------------------------------------------------------
// Internal state machine constants for the DART control-port protocol.
// The DART uses a two-phase write to its control port: the first byte
// selects the register number, the second byte writes the data.
// ---------------------------------------------------------------------------
static constexpr int REGMODE_REGISTER = 1;
static constexpr int REGMODE_DATA     = 2;

// ---------------------------------------------------------------------------
// Z80Dart — base class modelling one DART channel's register file and
// the two-phase control-port protocol.
// ---------------------------------------------------------------------------
class Z80Dart
{
protected:
    int m_regno;
    int m_regmode;

public:
    uint8_t m_latch;
    uint8_t m_reg[8];    // Write registers WR0–WR7
    uint8_t m_rreg[2];   // Read registers RR0–RR1

    void    reset();
    void    outCtrl(uint8_t b);
    uint8_t inCtrl();

    virtual void    outData(uint8_t b);
    virtual uint8_t inData();

protected:
    virtual uint8_t in (uint8_t reg);
    virtual void    out(uint8_t reg, uint8_t value);
};

// ---------------------------------------------------------------------------
// Z80DartA — Channel A, wired to the PcwComms serial interface.
//
// This maps the DART's write registers to the comms layer:
//   WR3  → RX bits-per-character
//   WR4  → parity and stop bits
//   WR5  → TX bits-per-character, RTS, DTR, BREAK
//
// And maps the comms layer status back into the DART's read registers:
//   RR0 bit 0 → RX character available
//   RR0 bit 2 → TX buffer empty
//   RR0 bit 5 → CTS
//   RR1 bit 0 → all sent
// ---------------------------------------------------------------------------
class Z80DartA : public Z80Dart
{
public:
    explicit Z80DartA(PcwComms *c) : m_comms(c) {}

    void    outData(uint8_t b) override;
    uint8_t inData()           override;

protected:
    uint8_t in (uint8_t reg)              override;
    void    out(uint8_t reg, uint8_t value) override;

private:
    PcwComms *m_comms;

    // Receive data latch — the RX side of the DART's data register, kept
    // SEPARATE from the base-class m_latch that outData() uses for the
    // transmit holding register.  On a real Z80 DART the TX and RX data
    // registers are physically distinct; sharing one latch caused a data
    // read taken while the RX FIFO was momentarily empty to return the last
    // TRANSMITTED byte, which made terminal software echo typed characters
    // back on screen.  See Z80DartA::inData().
    uint8_t   m_rxLatch = 0;
};

// ---------------------------------------------------------------------------
// Z80DartB — Channel B, wired to the Centronics printer port on a real
// CPS8256. Ported faithfully from JOYCE's Z80DartB.
//
// On the CPS8256 hardware:
//   RR0 bit 5 (CTS)  ← printer BUSY line (set when printer is NOT busy)
//   WR5 bit 7 (DTR)  → printer STROBE line (1→0 transition latches a
//                      character from the Centronics data latch, port 8)
//
// PerryFi has no physical Centronics port, but Channel B's register file
// must still exist and read back correctly: CP/M and application software
// talk to these ports, and a register bank that answers 0xFF to everything
// is indistinguishable from the interface being absent. RI/RTS (used only
// by the SCA Mk2 clock) are not implemented, exactly as in JOYCE.
// ---------------------------------------------------------------------------
class Z80DartB : public Z80Dart
{
public:
    explicit Z80DartB(CPS8256 *p) : m_printer(p) {}

protected:
    uint8_t in (uint8_t reg)                override;
    void    out(uint8_t reg, uint8_t value) override;

private:
    CPS8256 *m_printer;
};