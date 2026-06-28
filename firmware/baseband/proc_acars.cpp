/*
 * Copyright (C) 2014 Jared Boone, ShareBrained Technology, Inc.
 * Copyright (C) 2018 Furrtek
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "portapack_shared_memory.hpp"
#include "proc_acars.hpp"
#include "dsp_fir_taps.hpp"
#include "audio_dma.hpp"

#include "event_m4.hpp"

#include <cmath>
#include <complex>

#define SYN 0x16
#define SOH 0x01
#define STX 0x02
#define ETX 0x83
#define ETB 0x97
#define DLE 0x7f

ACARSProcessor::ACARSProcessor() {
    audio::dma::init_audio_out();
    decim_0.configure(taps_11k0_decim_0.taps);
    decim_1.configure(taps_11k0_decim_1.taps);
    init_msk();
    audio_output.configure(false);
    baseband_thread.start();
}

// Build the MSK matched filter and reset the demodulator state.
// msk_h is a half-cosine pulse at 600 Hz (the MSK half-symbol), clamped to
// non-negative and oversampled by msk_fltover so it can be sampled at the
// fractional bit-timing offset chosen by the PLL each bit.
void ACARSProcessor::init_msk() {
    for (int i = 0; i < msk_fleno; i++) {
        float v = cosf(2.0f * msk_pi * 600.0f / msk_intrate / msk_fltover *
                       (i - (msk_fleno - 1) / 2));
        msk_h[i] = (v < 0.0f) ? 0.0f : v;
    }
    msk_idx = 0;
    msk_osc = {1.0f, 0.0f};
    msk_clk = 0.0f;
    msk_df = 0.0f;
    msk_state = 0;
    msk_s = 2.0f * msk_pi * 1800.0f / msk_intrate;  // VCO at the 1800 Hz MSK centre
    msk_rot = {cosf(-msk_s), sinf(-msk_s)};
    for (auto& s : msk_inb)
        s = {0.0f, 0.0f};
}

void ACARSProcessor::execute(const buffer_c8_t& buffer) {
    /* 2.4576MHz, 2048 samples */

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
    const auto decimator_out = decim_1_out;

    /* 38.4kHz, 32 samples */
    feed_channel_stats(decimator_out);

    // AM detect -> real envelope carrying the 1200/2400 Hz MSK audio.
    const auto am_audio = demod.execute(decimator_out, audio_buffer);
    audio_output.write(am_audio);  // also routed to the speaker for tuning

    // Recover bits from the envelope and feed the framing state machine.
    demod_msk(am_audio);
}

// Coherent MSK demodulator (ported from acarsdec demodMSK, T. Leconte).
// Per sample: mix the AM envelope down by the 1800 Hz VCO into a ring buffer.
// Once per bit (every 3*pi/2 of VCO phase = one ACARS bit at 2400 bps): run the
// oversampled matched filter at the PLL-chosen timing offset, take the MSK
// decision (alternating I/Q rails), update the carrier/bit PLL, and hand the
// soft bit to consume_symbol().
void ACARSProcessor::demod_msk(const buffer_f32_t& audio_in) {
    for (size_t n = 0; n < audio_in.count; n++) {
        // VCO / mixer: advance oscillator, downconvert this sample.
        msk_osc *= msk_rot;
        const float in = audio_in.p[n];
        msk_inb[msk_idx] = in * msk_osc;
        msk_idx = (msk_idx + 1) % msk_flen;

        // Bit clock: a decision is due every 3*pi/2 of accumulated VCO phase.
        msk_clk += msk_s;
        if (msk_clk < (3.0f * msk_pi / 2.0f - msk_s / 2.0f))
            continue;
        msk_clk -= 3.0f * msk_pi / 2.0f;

        // Matched filter at the fractional timing offset for this bit.
        int o = static_cast<int>(msk_fltover * (msk_clk / msk_s + 0.5f));
        if (o > msk_fltover)
            o = msk_fltover;
        std::complex<float> v{0.0f, 0.0f};
        for (int j = 0; j < msk_flen; j++, o += msk_fltover)
            v += msk_h[o] * msk_inb[(j + msk_idx) % msk_flen];

        // Normalise to unit magnitude so the PLL error is amplitude-independent.
        const float lvl = std::abs(v);
        v /= (lvl + 1e-8f);

        // MSK decision: even bits ride the real rail, odd bits the imaginary
        // rail; dphi is the carrier-phase error fed to the PLL.
        float vo;
        float dphi;
        if (msk_state & 1) {
            vo = v.imag();
            dphi = (vo >= 0.0f) ? -v.real() : v.real();
        } else {
            vo = v.real();
            dphi = (vo >= 0.0f) ? v.imag() : -v.imag();
        }

        // Alternate the data-bit sign every other pair of decisions.
        float bit_soft = (msk_state & 2) ? -vo : vo;
        if (msk_invert)
            bit_soft = -bit_soft;
        consume_symbol(bit_soft);  // slices >= 0 -> bit 1, then runs framing
        msk_state++;

        // PLL loop filter, then refresh the VCO step and per-sample rotation.
        msk_df = msk_pllc * msk_df + (1.0f - msk_pllc) * msk_pllg * dphi;
        msk_s = 2.0f * msk_pi * 1800.0f / msk_intrate + msk_df;
        msk_rot = {cosf(-msk_s), sinf(-msk_s)};

        // Keep the mixing oscillator on the unit circle.
        msk_osc /= (std::abs(msk_osc) + 1e-8f);
    }
}

// Shift one demodulated bit into the byte assembler, LSB first (ACARS wire
// order). After 8 bits the byte sits in bits [7:0] and matches the real ACARS
// byte values used by the framing comparisons below.
void ACARSProcessor::add_bit(uint8_t bit) {
    decode_data = (decode_data >> 1) | (static_cast<uint32_t>(bit & 1) << 7);
    decode_count_bit++;
}

void ACARSProcessor::sendDebug() {
    // if (curr_state <= 1) return;
    message.state = curr_state;
    shared_memory.application_queue.push(message);
}

void ACARSProcessor::reset() {
    decode_data = 0;
    decode_count_bit = 0;
    curr_state = WSYN;
    message.msg_len = 0;
    memset(message.message, 0, 250);
    message.crc[0] = 0;
    message.crc[1] = 0;
    parity_errors = 0;
}

void ACARSProcessor::consume_symbol(const float raw_symbol) {
    const uint_fast8_t sliced_symbol = (raw_symbol >= 0.0f) ? 1 : 0;

    add_bit(sliced_symbol);
    if (curr_state == WSYN && decode_count_bit == 8) {
        if ((decode_data & 0xff) == SYN) {
            curr_state = SYN2;
            decode_data = 0;
            decode_count_bit = 0;
        } else {
            decode_count_bit -= 1;  // slide the search window by one bit
        }
        return;
    }
    if (curr_state == SYN2 && decode_count_bit == 8) {
        if ((decode_data & 0xff) == SYN) {
            curr_state = SOH1;
            decode_data = 0;
            decode_count_bit = 0;
            sendDebug();
            return;
        }
        // here i don't have the right packets. so reset
        reset();
    }
    if (curr_state == SOH1 && decode_count_bit == 8) {
        if ((decode_data & 0xff) == SOH) {
            reset();
            curr_state = TXT;
            sendDebug();
            return;
        }
        message.message[0] = (decode_data & 0xff);  // debug
        reset();
        sendDebug();
    }
    if (curr_state == TXT && decode_count_bit == 8) {
        uint8_t ch = (decode_data & 0xff);
        message.message[message.msg_len++] = ch;

        if (!ParityCheck::parity_check(ch)) {
            // parity error
            parity_errors++;
            if (parity_errors > 4) {
                reset();  // too many parity errors, skip packet
                sendDebug();
                return;
            }
        }

        if (ch == ETX || ch == ETB) {
            curr_state = CRC1;
            sendDebug();
            decode_data = 0;
            decode_count_bit = 0;
            return;
        }
        if (message.msg_len > 240) {
            reset();
            sendDebug();
        }
        if (message.msg_len > 20 && ch == DLE) {
            message.msg_len -= 3;
            message.crc[0] = message.message[message.msg_len];
            message.crc[1] = message.message[message.msg_len + 1];
            curr_state = CRC2;
            sendDebug();
            // to hack the path:
            decode_data = message.crc[1];
        } else {
            decode_count_bit = 0;
            decode_data = 0;
            return;
        }
    }
    if (curr_state == CRC1 && decode_count_bit == 8) {
        message.crc[0] = (decode_data & 0xff);
        curr_state = CRC2;
        decode_data = 0;
        decode_count_bit = 0;
        sendDebug();
    }

    if (curr_state == CRC2 && decode_count_bit == 8) {
        message.crc[1] = (decode_data & 0xff);
        // send it to app cpu, and it'll take care of the rest
        payload_handler();
        reset();
        curr_state = END;
        decode_data = 0;
        decode_count_bit = 0;
        sendDebug();
    }
    if (curr_state == END && decode_count_bit == 8) {
        reset();
        sendDebug();
    }
}

void ACARSProcessor::payload_handler() {
    message.state = 255;  // to indicate this is an actual payload, not a debug packet
    shared_memory.application_queue.push(message);
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<ACARSProcessor>()};
    event_dispatcher.run();
    return 0;
}
