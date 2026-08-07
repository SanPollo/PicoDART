/*
 * PicoDART - Amstrad CPS8256 emulator for the Raspberry Pi Pico
 *
 * Copyright 1996, 2001–2, 2020 John Elliott <seasip.webmaster@gmail.com>
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
 * PicoComms.cpp - Pico implementation of the PcwComms interface
 *
 */

#include "PicoComms.hpp"
#include "TraceUart.hpp"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Map an LL_PARITY value to the Pico SDK's uart_parity_t.
static uart_parity_t toPicoParity(LL_PARITY p)
{
    switch (p)
    {
        case PE_EVEN: return UART_PARITY_EVEN;
        case PE_ODD:  return UART_PARITY_ODD;
        default:      return UART_PARITY_NONE;
    }
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

PicoComms::PicoComms(uart_inst_t *uart, int defaultBaud)
    : m_uart(uart),
      m_dataBits(8),
      m_stopBits(1),
      m_parity(PE_NONE),
      m_txBaud(defaultBaud),
      m_rxBaud(defaultBaud),
      m_breakActive(false),
      m_breakEndUs(0)
{
    // Initialise the cross-core UART register lock first.  The constructor
    // itself runs before Core 1 is launched, so the configuration calls
    // below need no locking — but the lock must exist before both cores
    // are alive.
    critical_section_init(&m_uartLock);

    // Initialise the UART peripheral at the requested baud rate.
    uart_init(m_uart, defaultBaud);
    TraceUart::print("[COM] UART init baud=");
    TraceUart::printDec(static_cast<uint32_t>(defaultBaud));
    TraceUart::println();

    // Assign GPIO pins to the UART function.
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initialise CTSA (input) — driven by the ESP8266, read by getCTS().
    gpio_init(ESP8266_CTS_PIN);
    gpio_set_dir(ESP8266_CTS_PIN, GPIO_IN);
    gpio_disable_pulls(ESP8266_CTS_PIN);

    // Initialise DTRA (output) — driven by the Pico, read by the ESP8266.
    // DTR is active low; we deassert it (high) until the DART sets it.
    gpio_init(ESP8266_DTR_PIN);
    gpio_put(ESP8266_DTR_PIN, true);   // deasserted initially
    gpio_set_dir(ESP8266_DTR_PIN, GPIO_OUT);

    // Apply initial format: 8N1.
    applyFormat();

    // Disable hardware flow control — the ESP8266 RetroWiFiModem firmware
    // does not use CTS/RTS.
    uart_set_hw_flow(m_uart, false, false);

    // Enable FIFO so that uart_is_readable() / uart_is_writable() reflect
    // actual FIFO state rather than single-character shift register state.
    uart_set_fifo_enabled(m_uart, true);
}

PicoComms::~PicoComms()
{
    uart_deinit(m_uart);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void PicoComms::applyFormat()
{
    uart_set_format(m_uart, m_dataBits, m_stopBits, toPicoParity(m_parity));
}

void PicoComms::applyBaud()
{
    // The hardware UART uses one baud rate for both TX and RX.
    // We use the TX rate as authoritative; setRX() overrides it.
    const int rounded = roundToStandardBaud(m_txBaud);
    TraceUart::print("[COM] applyBaud ");
    TraceUart::printDec(static_cast<uint32_t>(m_txBaud));
    TraceUart::print(" -> ");
    TraceUart::printDec(static_cast<uint32_t>(rounded));
    TraceUart::println();
    uart_set_baudrate(m_uart, rounded);
}

int PicoComms::roundToStandardBaud(int baud)
{
    // The 8253 formula (125000 / divisor) produces non-standard values whose
    // error grows at higher rates: divisor 7 gives 17857 instead of 19200
    // (7% error), divisor 3 gives 41667 instead of 38400 (8.5% error), and
    // so on.  uart_set_baudrate() would set those exact wrong rates via the
    // RP2040's fractional dividers, causing framing errors at the ESP-12F.
    // PC serial drivers round to the nearest standard rate implicitly; we
    // must do it explicitly.
    //
    // Table covers all standard rates the PCW CPS8256 is expected to use.
    static constexpr int table[] = {
        300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
    };
    static constexpr int N = static_cast<int>(sizeof(table) / sizeof(table[0]));

    if (baud <= 0)
        return baud;

    int best     = table[0];
    int bestDiff = (baud > table[0]) ? (baud - table[0]) : (table[0] - baud);

    for (int i = 1; i < N; ++i)
    {
        int diff = (baud > table[i]) ? (baud - table[i]) : (table[i] - baud);
        if (diff < bestDiff)
        {
            bestDiff = diff;
            best     = table[i];
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// flush / reset
// ---------------------------------------------------------------------------

void PicoComms::flush()
{
    // Drain any pending RX characters from the FIFO.
    while (uart_is_readable(m_uart))
        uart_getc(m_uart);
}

void PicoComms::reset()
{
    // Return to 9600 8N1 and drain the FIFOs.
    m_dataBits = 8;
    m_stopBits = 1;
    m_parity   = PE_NONE;
    m_txBaud   = 9600;
    m_rxBaud   = 9600;

    critical_section_enter_blocking(&m_uartLock);
    applyFormat();
    applyBaud();
    flush();

    // Abandon any BREAK in progress.
    if (m_breakActive)
    {
        uart_set_break(m_uart, false);
        m_breakActive = false;
    }
    critical_section_exit(&m_uartLock);

    // Deassert DTR on reset.
    gpio_put(ESP8266_DTR_PIN, true);
}

// ---------------------------------------------------------------------------
// Transmit status
// ---------------------------------------------------------------------------

bool PicoComms::getTxBufferEmpty()
{
    // True when the TX FIFO has room for at least one more character.
    // This matches the DART RR0 bit 2 meaning ("TX buffer empty").
    return uart_is_writable(m_uart);
}

bool PicoComms::getCTS()
{
    // CTSA is active low — the ESP8266 pulls the line low to signal
    // clear-to-send.  We return true when the line is asserted (low).
    return !gpio_get(ESP8266_CTS_PIN);
}

bool PicoComms::getAllSent()
{
    // True when the TX FIFO is empty AND the shift register has finished
    // clocking out its last bit.  This maps to the UART BUSY flag being clear.
    //
    // The Pico SDK does not expose a direct "all sent" helper, so we read the
    // UARTFR (flag register) directly.  BUSY is bit 3.
    return !(uart_get_hw(m_uart)->fr & UART_UARTFR_BUSY_BITS);
}

// Assert BREAK, and return immediately.
//
// This is called from Core 1's bus handler, with /WAIT asserted and the Z80
// frozen. A previous version slept for 200 ms here, which would have
// stalled the PCW for that entire period. Instead we start the BREAK and let
// serviceBreak(), called from the Core 0 main loop, clear it when the time
// has elapsed.
void PicoComms::sendBreak()
{
    // Runs on Core 1.  uart_set_break() read-modify-writes LCR_H, and
    // serviceBreak() on Core 0 does the same to end the break — so both
    // sides take the UART lock.  m_breakEndUs is a 64-bit value written in
    // two halves on the Cortex-M0+, so it too must only be touched under
    // the lock, or Core 0 could read a torn value and end the break at the
    // wrong time.
    critical_section_enter_blocking(&m_uartLock);
    uart_set_break(m_uart, true);
    m_breakActive = true;
    m_breakEndUs  = time_us_64() + 200000u;   // 200 ms
    critical_section_exit(&m_uartLock);
}

void PicoComms::serviceBreak()
{
    // Unlocked fast-path peek (m_breakActive is volatile).  A stale value
    // here is harmless: it only defers the real, locked check to the next
    // main-loop pass, microseconds later.
    if (!m_breakActive)
        return;

    critical_section_enter_blocking(&m_uartLock);
    if (m_breakActive && time_us_64() >= m_breakEndUs)
    {
        uart_set_break(m_uart, false);
        m_breakActive = false;
    }
    critical_section_exit(&m_uartLock);
}

// ---------------------------------------------------------------------------
// Receive status / control
// ---------------------------------------------------------------------------

bool PicoComms::getRxCharacterAvailable()
{
    return uart_is_readable(m_uart);
}

void PicoComms::setDTR(bool b)
{
    // DTRA is active low — we drive the line low to assert DTR.
    TraceUart::print("[COM] DTR=");
    TraceUart::printDec(b ? 1u : 0u);
    TraceUart::println();
    gpio_put(ESP8266_DTR_PIN, !b);
}

void PicoComms::setRTS(bool /*b*/)
{
    // RTS is not wired to the ESP8266 in the PerryFi design.
    // If an RTS GPIO is added later, drive it here.
}

// ---------------------------------------------------------------------------
// Framing parameters
// ---------------------------------------------------------------------------

void PicoComms::setTXbits(int bits)
{
    // The JOYCE code passes 5–8 (derived from DART WR5 bits 6:5).
    // The Pico UART supports 5–8 data bits.
    // TX and RX data-bit widths are not independently configurable in the
    // RP2040 hardware, so we use the TX width for both.
    if (bits >= 5 && bits <= 8 && bits != m_dataBits)
    {
        TraceUart::print("[COM] TXbits=");
        TraceUart::printDec(static_cast<uint32_t>(bits));
        TraceUart::println();
        critical_section_enter_blocking(&m_uartLock);
        m_dataBits = bits;
        applyFormat();
        critical_section_exit(&m_uartLock);
    }
}

void PicoComms::setRXbits(int bits)
{
    // The JOYCE code passes 5–8 (derived from DART WR3 bits 7:6).
    // As above, the hardware cannot set RX and TX widths independently;
    // we update the shared setting if the value differs from current.
    if (bits >= 5 && bits <= 8 && bits != m_dataBits)
    {
        TraceUart::print("[COM] RXbits=");
        TraceUart::printDec(static_cast<uint32_t>(bits));
        TraceUart::println();
        critical_section_enter_blocking(&m_uartLock);
        m_dataBits = bits;
        applyFormat();
        critical_section_exit(&m_uartLock);
    }
}

void PicoComms::setStop(int stops)
{
    // The JOYCE code (Z80DartA::out) passes the values:
    //   2 → 1 stop bit
    //   3 → 1.5 stop bits  (not supported by RP2040 UART; treated as 2)
    //   4 → 2 stop bits
    int hwStops;
    switch (stops)
    {
        case 2:  hwStops = 1; break;
        case 3:  hwStops = 2; break;  // 1.5 not supported; round up
        case 4:  hwStops = 2; break;
        default: hwStops = m_stopBits; break;
    }

    if (hwStops != m_stopBits)
    {
        TraceUart::print("[COM] Stop=");
        TraceUart::printDec(static_cast<uint32_t>(hwStops));
        TraceUart::print(" (raw=");
        TraceUart::printDec(static_cast<uint32_t>(stops));
        TraceUart::print(")\r\n");
        critical_section_enter_blocking(&m_uartLock);
        m_stopBits = hwStops;
        applyFormat();
        critical_section_exit(&m_uartLock);
    }
}

void PicoComms::setParity(LL_PARITY p)
{
    if (p != m_parity)
    {
        TraceUart::print("[COM] Par=");
        switch (p)
        {
            case PE_NONE: TraceUart::print("N"); break;
            case PE_ODD:  TraceUart::print("O"); break;
            case PE_EVEN: TraceUart::print("E"); break;
        }
        TraceUart::println();
        critical_section_enter_blocking(&m_uartLock);
        m_parity = p;
        applyFormat();
        critical_section_exit(&m_uartLock);
    }
}

void PicoComms::setTX(int baud)
{
    // A real 8253 reloads its counter on every low+high write, even when the
    // new divisor equals the current one — there is no "unchanged" short
    // circuit in the silicon.  We mirror that here: any valid rate is applied
    // unconditionally.  (An earlier version skipped re-application when
    // baud == m_txBaud; because the Pico's cached rate persists across PCW
    // program restarts, that suppressed a freshly-launched program's INITMOD
    // when it re-asserted a rate the Pico had already been told earlier in the
    // session — so the hardware was never actually programmed.  Applying
    // unconditionally, as the 8253 does, removes that failure.)
    if (baud > 0)
    {
        TraceUart::print("[COM] setTX(");
        TraceUart::printDec(static_cast<uint32_t>(baud));
        TraceUart::print(")\r\n");
        critical_section_enter_blocking(&m_uartLock);
        m_txBaud = baud;
        // The RP2040 UART uses one baud rate for both directions.
        // Whichever of setTX() or setRX() was called most recently
        // determines the actual hardware rate.
        applyBaud();
        critical_section_exit(&m_uartLock);
    }
}

void PicoComms::setRX(int baud)
{
    // Applied unconditionally for the same reason as setTX() above: a real
    // 8253 reloads on every write, and the guard against baud == m_rxBaud
    // otherwise suppressed INITMOD re-programming after a program restart.
    if (baud > 0)
    {
        TraceUart::print("[COM] setRX(");
        TraceUart::printDec(static_cast<uint32_t>(baud));
        TraceUart::print(")\r\n");
        critical_section_enter_blocking(&m_uartLock);
        m_rxBaud = baud;
        // As above: last write wins.  applyBaud() is not used here because
        // it would apply m_txBaud; we write m_rxBaud directly instead.
        const int rounded = roundToStandardBaud(m_rxBaud);
        TraceUart::print("[COM] setbaud ");
        TraceUart::printDec(static_cast<uint32_t>(m_rxBaud));
        TraceUart::print(" -> ");
        TraceUart::printDec(static_cast<uint32_t>(rounded));
        TraceUart::println();
        uart_set_baudrate(m_uart, rounded);
        critical_section_exit(&m_uartLock);
    }
}

// ---------------------------------------------------------------------------
// Data transfer
// ---------------------------------------------------------------------------

void PicoComms::write(char c)
{
    // This function must NEVER block:
    //
    //   Called from Z80DartA::outData() on Core 1 with /WAIT asserted and
    //   the PCW's Z80 frozen.  uart_putc() spins while the TX FIFO is full;
    //   a full 32-byte FIFO takes ~33 ms to drain at 9600 baud — freezing
    //   the Z80 that long also stops the DRAM refresh it provides, which
    //   can corrupt the PCW's memory.
    //
    // Well-behaved software checks RR0 bit 2 ("TX buffer empty") before
    // writing — CP/M does — so in normal use the FIFO always has room.
    // If a writer ignores the status and the FIFO is full, discard the
    // byte instead of stalling: the real DART has a single TX buffer, and
    // an unchecked write simply clobbers the pending byte, so losing data
    // — not stalling the bus — is the hardware-faithful outcome.
    //
    // uart_putc_raw() is used rather than uart_putc() so that CR/LF
    // translation can never be applied to modem data, even if some future
    // change enables translation globally.
    if (uart_is_writable(m_uart))
        uart_putc_raw(m_uart, c);
}

char PicoComms::read()
{
    // uart_getc() blocks if no character is available.  The DART emulation
    // checks getRxCharacterAvailable() before calling this, so in normal
    // operation it will not block.
    return uart_getc(m_uart);
}