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
 * CPS8256.cpp
 *
 * Implementation of CPS8256, adapted from the JOYCE PCW emulator's
 * JoyceCPS.cxx for use on the Raspberry Pi Pico.
 *
 * Original source:
 *   JOYCE v2.2.13 - Amstrad PCW emulator
 *   Copyright (C) 1996, 2001, 2020  John Elliott <seasip.webmaster@gmail.com>
 *   https://www.seasip.info/Unix/Joyce/
 *
 */

#include "CPS8256.hpp"
#include "TraceUart.hpp"

// ---------------------------------------------------------------------------
// Constructor / reset
// ---------------------------------------------------------------------------

CPS8256::CPS8256(PcwComms *comms)
    : m_comms(comms),
      m_dartA(comms),
      m_dartB(this),
      m_cench(-1),
      m_baudmode(0),
      m_txbaud(9600),
      m_rxbaud(9600),
      m_baudbuf(0),
      m_pollTick(0)
{
    reset();
}

void CPS8256::reset()
{
    m_baudmode = 0;
    m_txbaud   = 9600;
    m_rxbaud   = 9600;
    m_baudbuf  = 0;
    m_pollTick = 0;
    m_cench    = -1;
    m_dartA.reset();
    m_dartB.reset();
}

// ---------------------------------------------------------------------------
// Bus read
// ---------------------------------------------------------------------------

uint8_t CPS8256::in(uint8_t port)
{
    switch (port & 0x0F)
    {
        case 0: return m_dartA.inData();
        case 1: return m_dartA.inCtrl();
        case 2: return m_dartB.inData();
        case 3: return m_dartB.inCtrl();
    }
    return 0xFF;
}

// ---------------------------------------------------------------------------
// Bus write
// ---------------------------------------------------------------------------

void CPS8256::out(uint8_t port, uint8_t value)
{
    switch (port & 0x0F)
    {
        case 0: m_dartA.outData(value); break;
        case 1: m_dartA.outCtrl(value); break;
        case 2: m_dartB.outData(value); break;
        case 3: m_dartB.outCtrl(value); break;

        // ------------------------------------------------------------------
        // Ports 4 and 5: 8253 timer data register.
        //
        // The PCW programs the 8253 with a 16-bit divisor in two successive
        // byte writes (low byte first, then high byte).  The baud rate is:
        //
        //     baud = 125000 / divisor
        //
        // The state machine in m_baudmode handles the sequencing:
        //   1 → waiting for TX low byte
        //   2 → waiting for TX high byte  (calculates and applies TX baud)
        //   3 → waiting for RX low byte
        //   4 → waiting for RX high byte  (calculates and applies RX baud)
        // ------------------------------------------------------------------
        case 4:
        case 5:
            switch (m_baudmode)
            {
                case 1:
                case 3:
                    // Low byte of divisor — stash it and advance.
                    m_baudbuf = value;
                    ++m_baudmode;
                    TraceUart::print("[CPS] Low=");
                    TraceUart::printHex(value, 2);
                    TraceUart::println();
                    break;

                case 2:
                case 4:
                {
                    // High byte of divisor — compute and apply baud rate.
                    unsigned int divisor = (static_cast<unsigned int>(value) << 8)
                                          | static_cast<unsigned int>(m_baudbuf);
                    unsigned int baud = (divisor != 0) ? (125000u / divisor) : 0u;

                    TraceUart::print("[CPS] Hi=");
                    TraceUart::printHex(value, 2);
                    TraceUart::print(" Div=");
                    TraceUart::printHex(divisor, 4);
                    TraceUart::print(" Baud=");
                    TraceUart::printDec(baud);
                    TraceUart::println();

                    if (m_baudmode == 2)
                    {
                        m_txbaud = static_cast<int>(baud);
                        TraceUart::print("[CPS] =>setTX(");
                        TraceUart::printDec(static_cast<uint32_t>(m_txbaud));
                        TraceUart::print(")\r\n");
                        m_comms->setTX(m_txbaud);
                    }
                    else
                    {
                        m_rxbaud = static_cast<int>(baud);
                        TraceUart::print("[CPS] =>setRX(");
                        TraceUart::printDec(static_cast<uint32_t>(m_rxbaud));
                        TraceUart::print(")\r\n");
                        m_comms->setRX(m_rxbaud);
                    }
                    m_baudmode = 0;
                    break;
                }

                default:
                    // Baudmode 0: no timer programming sequence in progress.
                    // This write is unexpected; ignore it.
                    break;
            }
            break;

        // ------------------------------------------------------------------
        // Port 7: 8253 timer control word.
        //
        // The PCW only ever writes two values here:
        //   0x36 → program counter 0 (TX baud rate): mode 3, 16-bit binary
        //   0x76 → program counter 1 (RX baud rate): mode 3, 16-bit binary
        //
        // Any other value is unexpected; we ignore it silently.
        // ------------------------------------------------------------------
        case 7:
            if (value == 0x36)
            {
                m_baudmode = 1;   // TX divisor incoming
                TraceUart::print("[CPS] 8253 ctrl=");
                TraceUart::printHex(value, 2);
                TraceUart::print(" TX\r\n");
            }
            else if (value == 0x76)
            {
                m_baudmode = 3;   // RX divisor incoming
                TraceUart::print("[CPS] 8253 ctrl=");
                TraceUart::printHex(value, 2);
                TraceUart::print(" RX\r\n");
            }
            // else: unexpected control word — ignore
            break;

        // Port 8: Centronics character latch. Store it; it is committed to
        // the (virtual) printer when Channel B drops STROBE — see
        // Z80DartB::out() and CPS8256::dropStrobe().
        case 8: m_cench = value; break;
    }
}

// ---------------------------------------------------------------------------
// poll() — interrupt-mode support
//
// The DART's WR1 bits 4:3 select the interrupt mode:
//   0x00 = polled mode (most CP/M comms software, including QTERM)
//   0x18 = interrupt on RX character available
//
// This method should be called regularly from the main loop.  It returns
// true when the DART is in interrupt mode and a received character is
// waiting, signalling that the caller should assert /INT to the PCW's Z80.
//
// The call is throttled internally so that checking the comms layer on
// every single iteration of the main loop is avoided.
// ---------------------------------------------------------------------------
bool CPS8256::poll()
{
    // Only check every 100 calls, matching the original JOYCE behaviour.
    // On the Pico's 125 MHz main loop this is still very fast relative to
    // serial baud rates.
    if (++m_pollTick < 100)
        return false;
    m_pollTick = 0;

    // Is Channel A configured for interrupt-driven mode?
    if (!(m_dartA.m_reg[1] & 0x18))
        return false;

    // Is a character waiting?
    //
    // IMPORTANT: this method must have NO side effects on the DART's
    // registers. It runs on Core 0, while Z80DartA::in() runs on Core 1.
    // A previous version did `m_rreg[0] |= 0x03` here, a non-atomic
    // read-modify-write on the very byte that Core 1's in(0) is also
    // read-modify-writing. Interleaving could leave RR0 bit 0 ("receive
    // character available") set when no character was actually waiting,
    // causing CP/M to read port E0h and be handed a stale byte.
    //
    // Z80DartA::in(0) already recomputes RR0's live bits from the comms
    // layer on every read, so it is the single writer and no update is
    // needed here.
    return m_comms->getRxCharacterAvailable();
}