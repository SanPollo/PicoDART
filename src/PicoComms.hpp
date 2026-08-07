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
 * PicoComms.hpp - Pico implementation of the PcwComms interface
 *
 * This replaces PcwCommsUnix.cxx / PcwCommsW32.cxx from the JOYCE emulator.
 * It drives one of the Pico's hardware UARTs to communicate with the
 * ESP8266 (WeMos D1 mini) Wi-Fi modem module.
 *
 */

#pragma once

#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// Parity enum — matches the LL_PARITY definition from the JOYCE emulator's
// LibLink/include/liblink.h, reproduced here so we do not depend on LibLink.
//
// liblink.h carries its own copyright, separate from the rest of JOYCE:
//   Copyright (C) 2002, 2023 John Elliott <seasip.webmaster@gmail.com>
// and is licensed under the GNU Library General Public License (LGPL),
// rather than the GPL that covers the rest of JOYCE.
// ---------------------------------------------------------------------------
typedef enum { PE_NONE, PE_EVEN, PE_ODD } LL_PARITY;

// ---------------------------------------------------------------------------
// GPIO pin assignments are defined centrally in config.hpp.
// ---------------------------------------------------------------------------
#include "config.hpp"

// ---------------------------------------------------------------------------
// PcwComms — abstract interface (extracted from JOYCE's PcwComms.hxx).
// Reproduced here so we do not need to include any JOYCE headers.
// ---------------------------------------------------------------------------
class PcwComms
{
protected:
    PcwComms() {}
public:
    virtual void flush()   = 0;
    virtual void reset()   = 0;

    // Transmit status
    virtual bool getTxBufferEmpty()       = 0;
    virtual bool getCTS()                 = 0;
    virtual bool getAllSent()             = 0;
    virtual void sendBreak()             = 0;

    // Receive status / control
    virtual bool getRxCharacterAvailable() = 0;
    virtual void setDTR(bool b)           = 0;
    virtual void setRTS(bool b)           = 0;

    // Framing parameters
    virtual void setTXbits(int bits)      = 0;
    virtual void setRXbits(int bits)      = 0;
    virtual void setStop  (int stops)     = 0;
    virtual void setParity(LL_PARITY p)   = 0;
    virtual void setTX(int baud)          = 0;
    virtual void setRX(int baud)          = 0;

    // Data transfer
    virtual void write(char c)            = 0;
    virtual char read()                   = 0;

    virtual ~PcwComms() {}
};

// ---------------------------------------------------------------------------
// PicoComms — concrete implementation for the RP2040 hardware UART
// ---------------------------------------------------------------------------
class PicoComms : public PcwComms
{
public:
    // Construct with an explicit uart instance (uart0 or uart1) and the
    // default baud rate.  The Pico SDK is assumed to be initialised before
    // this constructor is called.
    explicit PicoComms(uart_inst_t *uart = uart0,
                       int defaultBaud   = 9600);

    ~PicoComms() override;

    // PcwComms interface -------------------------------------------------------

    // Flush / reset
    void flush() override;
    void reset() override;

    // Transmit status
    bool getTxBufferEmpty()        override;
    bool getCTS()                  override;
    bool getAllSent()               override;
    void sendBreak()               override;

    // Receive status / control
    bool getRxCharacterAvailable() override;
    void setDTR(bool b)            override;  // drives ESP8266_DTR_PIN
    void setRTS(bool b)            override;  // no-op: not wired to ESP8266

    // Framing
    void setTXbits(int bits)       override;
    void setRXbits(int bits)       override;
    void setStop  (int stops)      override;
    void setParity(LL_PARITY p)    override;
    void setTX(int baud)           override;
    void setRX(int baud)           override;

    // Data transfer
    void write(char c)             override;
    char read()                    override;

    // Called regularly from the Core 0 main loop. Clears a BREAK condition
    // once its duration has elapsed — see sendBreak().
    void serviceBreak();

private:
    // Apply the current framing settings (data bits, stop bits, parity) to
    // the hardware UART.  Called whenever any framing parameter changes.
    void applyFormat();

    // Apply the current baud rate to the hardware UART.  Called whenever
    // setTX() or setRX() is called.
    void applyBaud();

    // Map a computed baud value (125000 / 8253-divisor) to the nearest
    // standard rate before passing it to uart_set_baudrate().
    //
    // The formula produces non-standard values at higher rates: divisor 7
    // gives 17857 where 19200 is expected (7% error — well outside the ±2%
    // UART framing tolerance).  PC serial drivers round silently; we must do
    // it explicitly.  Supported range: 300 – 115200 baud.
    static int roundToStandardBaud(int baud);

    uart_inst_t *m_uart;

    // Cached framing state — we only call uart_set_format() when something
    // actually changes, to avoid unnecessary hardware writes.
    int        m_dataBits;    // 5–8
    int        m_stopBits;    // 1 or 2 (the Pico UART does not support 1.5)
    LL_PARITY  m_parity;

    // The CPS8256 / 8253 can theoretically set TX and RX baud rates
    // independently, but the Pico's hardware UART uses a single baud rate
    // for both directions.  We honour whichever rate was set most recently.
    int m_txBaud;
    int m_rxBaud;

    // Deferred BREAK state — see sendBreak() / serviceBreak().
    // m_breakActive is volatile because serviceBreak() (Core 0) peeks at it
    // outside the lock as a fast path; it is only ever *changed* while
    // m_uartLock is held.  m_breakEndUs is only touched under the lock.
    volatile bool m_breakActive;
    uint64_t      m_breakEndUs;

    // Cross-core UART register lock.
    //
    // The DART emulation (Core 1) reconfigures the UART — uart_set_format()
    // and uart_set_break() read-modify-write LCR_H, and uart_set_baudrate()
    // writes the divisor and touches LCR_H to latch it — while
    // serviceBreak() (Core 0) also read-modify-writes LCR_H to end a BREAK.
    // Unguarded, those RMWs can interleave and tear the register (wrong
    // word length, a stuck BREAK bit, or a half-latched divisor).  The
    // window is small but lines up with real usage: software that sends a
    // BREAK often reconfigures the port straight afterwards.
    //
    // Invariant: every write to a UART *control* register (format, break,
    // baud divisor) takes this lock, on whichever core it runs.  Data-path
    // access (DR) and status reads (FR) are single-register operations and
    // do not need it.  The lock is never held across anything that blocks.
    critical_section_t m_uartLock;
};