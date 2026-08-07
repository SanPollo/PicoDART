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
 * Z80Dart.cpp
 *
 * Implementation of Z80Dart and Z80DartA, adapted from the JOYCE PCW
 * emulator's JoyceCPS.cxx for use on the Raspberry Pi Pico.
 *
 * Original source:
 *   JOYCE v2.2.13 - Amstrad PCW emulator
 *   Copyright (C) 1996, 2001, 2020  John Elliott <seasip.webmaster@gmail.com>
 *   https://www.seasip.info/Unix/Joyce/
 *
 */

#include "Z80Dart.hpp"
#include "CPS8256.hpp"   // Z80DartB reports to its owning CPS8256

// ---------------------------------------------------------------------------
// Convenience macro: true if the new value 'value' differs from the cached
// value in register n, masked by m.  Used in Z80DartA::out() to avoid
// redundant calls to the comms layer.
// ---------------------------------------------------------------------------
#define COMP(n, m)  ((value & (m)) != (m_reg[(n)] & (m)))

// ===========================================================================
// Z80Dart — base channel implementation
// ===========================================================================

void Z80Dart::reset()
{
    m_regmode = REGMODE_REGISTER;
    m_regno   = 0;
    m_latch   = 0;
    memset(m_reg,  0, sizeof(m_reg));
    memset(m_rreg, 0, sizeof(m_rreg));
}

// Two-phase control-port write.
// Phase 1: write selects the register number (low 3 bits).
//          If register 0 is selected the command byte is processed immediately.
// Phase 2: write sends the data to the previously selected register.
void Z80Dart::outCtrl(uint8_t b)
{
    switch (m_regmode)
    {
        case REGMODE_REGISTER:
            m_regno = (b & 7);
            if (m_regno) m_regmode = REGMODE_DATA;
            else         out(0, b);
            break;

        case REGMODE_DATA:
            out(m_regno, b);
            m_regmode = REGMODE_REGISTER;
            break;
    }
}

// Control-port read: return the currently selected read register.
uint8_t Z80Dart::inCtrl()
{
    switch (m_regmode)
    {
        case REGMODE_REGISTER:
            return in(0);

        case REGMODE_DATA:
            m_regmode = REGMODE_REGISTER;
            return in(m_regno);
    }
    return 0xFF;
}

uint8_t Z80Dart::inData()
{
    return m_latch;
}

void Z80Dart::outData(uint8_t b)
{
    m_latch = b;
}

// Default register write.  WR0 command codes are handled here;
// all other registers are just stored.
void Z80Dart::out(uint8_t reg, uint8_t value)
{
    if (reg == 0)
    {
        switch (value >> 3)
        {
            case 0: break;   // Null command
            case 1: break;   // Not used
            case 3: reset(); break;  // Channel reset
            // Commands 2, 4–7 (interrupt control, error reset, RETI) are
            // not needed in polled mode and are silently ignored.
            default: break;
        }
    }
    m_reg[reg & 7] = value;
}

// Default register read.
uint8_t Z80Dart::in(uint8_t reg)
{
    if (reg <  2) return m_rreg[reg];
    if (reg == 2) return m_reg[2];   // WR2 is readable as RR2
    return 0xFF;
}

// ===========================================================================
// Z80DartA — Channel A, wired to the serial comms layer
// ===========================================================================

// Read a DART status register, refreshing the live status bits from the
// comms layer before returning.
uint8_t Z80DartA::in(uint8_t reg)
{
    switch (reg)
    {
        case 0:
            // RR0: clear the live bits then repopulate from comms status.
            //   Bit 0: RX character available
            //   Bit 2: TX buffer empty
            //   Bit 5: CTS (active-high in RR0)
            m_rreg[0] &= ~0x25u;
            if (m_comms->getRxCharacterAvailable()) m_rreg[0] |= 0x01;
            if (m_comms->getTxBufferEmpty())        m_rreg[0] |= 0x04;
            if (m_comms->getCTS())                  m_rreg[0] |= 0x20;
            break;

        case 1:
            // RR1 bit 0: all sent (TX shift register empty).
            //
            // Clear bit 0 first, then refresh it from the live comms
            // status, leaving RR1's other bits untouched.  Inverting the
            // mask (m_rreg[1] &= 0x01) would be wrong: that
            // PRESERVES the old bit 0 and clears everything else — so once
            // "all sent" had ever read true it could never read false
            // again.  Software that polls RR1 bit 0 to wait for the
            // transmitter to drain (typically just before changing baud
            // rate or framing) would sail straight through and reconfigure
            // the UART mid-character, corrupting the byte in flight.
            m_rreg[1] &= static_cast<uint8_t>(~0x01u);
            if (m_comms->getAllSent()) m_rreg[1] |= 0x01;
            break;
    }
    return Z80Dart::in(reg);
}

// Write to a DART control register, passing any changed framing or control
// parameters through to the comms layer.
void Z80DartA::out(uint8_t reg, uint8_t value)
{
    if (reg == 5)
    {
        // WR5: TX parameters and control.
        //   Bit  1: RTS
        //   Bit  4: SEND BREAK (send break when this bit transitions 0→1)
        //   Bits 6:5: TX bits per character (00=5, 01=6, 10=7, 11=8)
        //   Bit  7: DTR
        if (COMP(5, 0x02)) m_comms->setRTS(value & 0x02);
        if (COMP(5, 0x10) && (value & 0x10)) m_comms->sendBreak();
        if (COMP(5, 0x60)) m_comms->setTXbits(((value >> 5) & 3) + 5);
        if (COMP(5, 0x80)) m_comms->setDTR(value & 0x80);
    }

    if (reg == 4)
    {
        // WR4: clock mode, stop bits, parity.
        //   Bits 1:0: parity enable/select
        //     00 or 10 = none, 01 = odd, 11 = even
        //   Bits 3:2: stop bits
        //     01 = 1, 10 = 1.5, 11 = 2  (00 = sync mode, unused here)
        //   Bits 7:6: clock multiplier (x1/x16/x32/x64) — not used on Pico
        if (COMP(4, 0x03))
        {
            switch (value & 0x03)
            {
                case 0: case 2: m_comms->setParity(PE_NONE); break;
                case 1:         m_comms->setParity(PE_ODD);  break;
                case 3:         m_comms->setParity(PE_EVEN); break;
            }
        }
        if (COMP(4, 0x0C))
        {
            switch (value & 0x0C)
            {
                case 0x04: m_comms->setStop(2); break;  // 1 stop bit
                case 0x08: m_comms->setStop(3); break;  // 1.5 stop bits
                case 0x0C: m_comms->setStop(4); break;  // 2 stop bits
            }
        }
    }

    if (reg == 3)
    {
        // WR3: RX parameters and control.
        //   Bit  0: RX enable
        //   Bits 7:6: RX bits per character (00=5, 01=6, 10=7, 11=8)
        if (COMP(3, 0xC0)) m_comms->setRXbits(((value >> 6) & 3) + 5);
    }

    // WR1 bits 4:3 select polled (00) or interrupt (0x18) mode.
    // This is reflected in m_reg[1] by the base-class call below and is
    // read by CPS8256::poll() to decide whether to assert /INT.

    Z80Dart::out(reg, value);
}

// Read one character from the comms layer into the latch, then return it.
// If no character is waiting, return the previous latch contents rather
// than calling m_comms->read() — which would block Core 1 forever with
// /WAIT asserted and freeze the PCW. A real DART also returns the last
// received character when read with nothing new, so this matches hardware.
uint8_t Z80DartA::inData()
{
    // Read one received character into the RECEIVE latch, then return it.
    // If nothing is waiting, return the previous receive-latch contents
    // rather than calling m_comms->read() — which would block Core 1 with
    // /WAIT asserted and freeze the PCW.
    //
    // m_rxLatch is deliberately SEPARATE from the base-class m_latch that
    // outData() writes for transmit.  A real Z80 DART has independent TX and
    // RX data registers: reading the data port yields received data, never
    // the last byte transmitted.  When the two shared a single latch, a
    // speculative data-port read taken with the RX FIFO momentarily empty
    // returned the host's own last-transmitted character, so terminal
    // programs (e.g. QTERM) saw typed characters fed back and repeated on
    // screen — behaviour a real DART never exhibits.  Keeping the latches
    // separate restores hardware-faithful operation.
    if (m_comms->getRxCharacterAvailable())
        m_rxLatch = m_comms->read();
    return m_rxLatch;
}

// Write the latch to the comms layer.
void Z80DartA::outData(uint8_t b)
{
    Z80Dart::outData(b);
    m_comms->write(m_latch);
}

// ===========================================================================
// Z80DartB — Channel B, the Centronics printer channel
//
// Ported faithfully from JOYCE's Z80DartB (JoyceCPS.cxx):
//   - CTS (RR0 bit 5) reflects the printer BUSY line: bit set when the
//     printer is NOT busy.
//   - DTR (WR5 bit 7) is the printer STROBE line. When a register write
//     causes STROBE to drop from 1 to 0, the character sitting in the
//     Centronics data latch (port 8) is committed to the printer.
//   - All other behaviour (register storage, WR2/RR2 vector read-back,
//     the two-phase control protocol) comes from the Z80Dart base class.
// ===========================================================================

uint8_t Z80DartB::in(uint8_t reg)
{
    if (reg == 0)
    {
        // Refresh CTS (bit 5) from the printer status before returning RR0.
        m_rreg[0] &= ~0x20u;
        m_rreg[0] |= (m_printer->isBusy() ? 0x00 : 0x20);
    }
    return Z80Dart::in(reg);
}

void Z80DartB::out(uint8_t reg, uint8_t value)
{
    // Capture STROBE (WR5 bit 7) before the write, then apply the write,
    // then check for a 1→0 transition — same ordering as JOYCE.
    uint8_t strobe = m_reg[5] & 0x80;
    Z80Dart::out(reg, value);

    if (reg == 5 && strobe)
    {
        if ((m_reg[5] & 0x80) != strobe)
            m_printer->dropStrobe();
    }
}