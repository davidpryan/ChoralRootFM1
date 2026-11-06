/* SPDX-License-Identifier: GPL-3.0-only
 * SMK-37 port: Copyright (C) 2026 M. Geisler (genkigenki/makkuro); from Vector37's firmware/src/smk_midi_out.h,
 * brought to ChoralRoot's SMK-37 build 2026-11-05 (SMK37.md "TRS MIDI OUT") */
/* SMK-37: TRS MIDI OUT, the same stream as USB MIDI out (usb.c midi_out_event hands every packet here, with or
 * without a computer): the CHORD and BASS streams on their channels (Options > MIDI Perform / MIDI Bass, ch 1 / 2 by
 * default), the raw chord when it is on, the wheels' bend and CC 1, the pedal's CC 64 / CC 11, PANIC's CC 123, and
 * MIDI clock with its start and stop when Options > MIDI Clock is Out. An FM-1 running LOOM-8 on the other end of the
 * cable plays the chords on its track 1 and the bass on track 2 (its tracks listen on channels 1..8).
 * A USB-MIDI packet becomes its MIDI bytes (no running status: every message whole), a whole message or nothing goes
 * into a 256-byte ring (producer: the audio ISR, as midi_out_event), and the TIMER5 tick (main.c fm1_timer5_irq, 10
 * kHz, nested in the render too) sends one byte whenever the UART has finished the last: ~0.32 ms a byte, never
 * waiting. A byte the UART does not finish within SMO_TIMEOUT ticks is given up; after SMO_DEAD of those in a row the
 * jack stays quiet until Options > TRS MIDI Out is set again (no UART: no hang, no stall). Options > TRS MIDI Out Off:
 * nothing is queued (cr_ui.c O_TRS, smo_on). Hardware: hal/smk_uart_tx.h (UART1 TX on PC7); the host test
 * (tests/smk_midi_out_test.c) and the browser emulator (tools/emu/emu_smk_hal.h) give SMO_TX_INIT / SMO_TX_DONE /
 * SMO_TX_PUT their own UART. Not in the FM-1 build (its TRS jack is MIDI IN: midi_uart.c). */
#pragma once
#include <stdint.h>

#ifndef SMO_TX_DONE
#define SMO_TX_INIT() smk_uart1_tx_init()
#define SMO_TX_DONE() smk_uart1_tx_done()
#define SMO_TX_PUT(b) smk_uart1_tx_put(b)
#endif
#ifndef SMO_PUBLISH
#define SMO_PUBLISH() __asm__ volatile("" ::: "memory")    /* the bytes before the index (one core: the audio ISR) */
#endif
#define SMO_RING 256u
#define SMO_TIMEOUT 30u           /* ticks (3 ms; a byte takes 0.32 ms) */
#define SMO_DEAD 4u               /* bytes timed out in a row: the UART is not there */

static uint8_t trs_out = 1;       /* Options > TRS MIDI Out (cr_ui.c cs.smk_trs mirrors it; On at power-on until the
                                   * session says Off: cr_settings.c css_apply) */
static struct {
    uint8_t ring[SMO_RING];
    volatile uint32_t w, r;       /* w: the producer's, r: the tick's */
    uint32_t wait;                /* ticks since the byte in flight started (0: none in flight) */
    uint32_t lost;                /* bytes timed out in a row */
    uint32_t sent, timeouts, drops;
    volatile uint8_t dead;
} smo;

static void smo_init(void) { SMO_TX_INIT(); }          /* main.c: before TIMER5 starts */

/* a USB-MIDI event packet (byte 0: cable << 4 | CIN, then up to three MIDI bytes) -> its MIDI bytes; 0: not sent
 * (SysEx, a status that does not fit its CIN, a data byte with bit 7) */
static uint32_t smo_bytes(uint32_t pkt, uint8_t *b)
{
    uint32_t cin = pkt & 15u, st = (pkt >> 8) & 255u, n, i;
    b[0] = (uint8_t)st, b[1] = (uint8_t)(pkt >> 16), b[2] = (uint8_t)(pkt >> 24);
    if (cin >= 8u && cin <= 14u) {                     /* channel voice: the status must be CIN's */
        if (st >> 4 != cin)
            return 0;
        n = cin == 12u || cin == 13u ? 2u : 3u;
    } else if (cin == 15u) {                           /* a single byte: realtime only */
        return st >= 0xF8u && st != 0xF9u && st != 0xFDu ? 1u : 0u;
    } else if (cin == 2u || cin == 3u) {               /* system common: F1, F3 / F2 */
        if (!(cin == 2u ? st == 0xF1u || st == 0xF3u : st == 0xF2u))
            return 0;
        n = cin;
    } else {
        return 0;
    }
    for (i = 1; i < n; i++)
        if (b[i] & 0x80u)
            return 0;
    return n;
}

static void smo_put(uint32_t pkt)                      /* producer: one packet, whole or dropped */
{
    uint8_t b[3];
    uint32_t n = smo_bytes(pkt, b), w = smo.w, i;
    if (!trs_out || smo.dead || !n)
        return;
    if (SMO_RING - (w - smo.r) < n) {
        smo.drops++;
        return;
    }
    for (i = 0; i < n; i++)
        smo.ring[(w + i) % SMO_RING] = b[i];
    SMO_PUBLISH();
    smo.w = w + n;
}

static void smo_tick(void)                             /* the TIMER5 tick: at most one byte, never waits */
{
    if (smo.wait) {                                    /* a byte in flight */
        if (!SMO_TX_DONE()) {
            if (++smo.wait <= SMO_TIMEOUT)
                return;
            smo.timeouts++;
            if (++smo.lost >= SMO_DEAD) {
                smo.dead = 1;                          /* no UART: quiet until set again (smo_on) */
                smo.r = smo.w;
            }
        } else {
            smo.lost = 0;
            smo.sent++;
        }
        smo.wait = 0;
    }
    if (smo.dead || smo.r == smo.w)
        return;
    SMO_TX_PUT(smo.ring[smo.r % SMO_RING]);
    smo.r++;
    smo.wait = 1;
}

static void smo_on(uint32_t on)                        /* Options > TRS MIDI Out On / Off (On tries the UART again) */
{
    trs_out = (uint8_t)(on != 0u);
    smo.lost = 0;
    smo.dead = 0;
}
