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

#ifndef __PROC_ACARS_H__
#define __PROC_ACARS_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"

#include "dsp_decimate.hpp"

#include "spectrum_collector.hpp"

#include "channel_decimator.hpp"
#include "matched_filter.hpp"

#include "clock_recovery.hpp"
#include "symbol_coding.hpp"
#include "packet_builder.hpp"
#include "baseband_packet.hpp"

#include "message.hpp"
#include "dsp_demodulate.hpp"
#include "audio_output.hpp"

#include "crc.hpp"

#include <array>
#include <complex>
#include <cstdint>
#include <cstddef>

// ACARS bit-framing states: wait for SYN/SYN/SOH preamble, accumulate text,
// then read the two CRC bytes.
enum Acarsstate { WSYN,
                  SYN2,
                  SOH1,
                  TXT,
                  CRC1,
                  CRC2,
                  END };

class ACARSProcessor : public BasebandProcessor {
   public:
    ACARSProcessor();

    void execute(const buffer_c8_t& buffer) override;

   private:
    static constexpr size_t baseband_fs = 2457600;

    // ---- Front end: translate + decimate to 38.4 kHz, then AM detect ----
    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{
        dst.data(),
        dst.size()};

    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};  // Translate already done here !
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};

    std::array<float, 32> audio{};
    const buffer_f32_t audio_buffer{
        audio.data(),
        audio.size()};
    dsp::demodulate::AM demod{};
    AudioOutput audio_output{};

    // ---- MSK demodulator ----
    // ACARS is 2400 bps MSK (1200/2400 Hz tones, 1800 Hz center) carried as AM.
    // The AM envelope produced by 'demod' is fed to a coherent MSK demodulator
    // with a combined carrier / bit-clock PLL. The algorithm is ported from
    // acarsdec demodMSK() (Copyright (c) 2017 Thierry Leconte, LGPL v2,
    // https://github.com/TLeconte/acarsdec), adapted to single-precision float
    // and to this 38.4 kHz front end. Constants that may need on-air tuning are
    // grouped below.
    static constexpr int msk_intrate = 38400;                     // = audio sample rate
    static constexpr int msk_flen = msk_intrate / 1200 + 1;       // matched-filter span (samples)
    static constexpr int msk_fltover = 12;                        // matched-filter oversampling
    static constexpr int msk_fleno = msk_flen * msk_fltover + 1;  // oversampled filter length
    static constexpr float msk_pi = 3.14159265358979f;
    static constexpr float msk_pllc = 0.52f;   // PLL loop pole (tunable)
    static constexpr float msk_pllg = 38e-4f;  // PLL loop gain (tunable)
    // Set to true if the recovered bit stream is inverted for your receiver
    // (signal clearly present but no SYN lock at all): flip and rebuild.
    static constexpr bool msk_invert = false;

    std::array<float, msk_fleno> msk_h{};                 // matched filter (clamped half-cosine)
    std::array<std::complex<float>, msk_flen> msk_inb{};  // mixer ring buffer
    int msk_idx = 0;
    std::complex<float> msk_osc{1.0f, 0.0f};  // mixing oscillator (kept at |.| = 1)
    std::complex<float> msk_rot{1.0f, 0.0f};  // per-sample rotation = e^(-j*msk_s)
    float msk_s = 0.0f;                       // VCO step (rad/sample)
    float msk_clk = 0.0f;                     // bit-clock phase accumulator
    float msk_df = 0.0f;                      // PLL frequency correction
    int msk_state = 0;                        // MSK I/Q decision phase (0..3)

    void init_msk();
    void demod_msk(const buffer_f32_t& audio_in);

    // ---- Bit framing / packet assembly ----
    Acarsstate curr_state = WSYN;
    uint32_t decode_data = 0;
    uint8_t decode_count_bit = 0;
    ACARSPacketMessage message{};
    uint8_t parity_errors = 0;

    void consume_symbol(const float symbol);
    void add_bit(uint8_t bit);
    void payload_handler();
    void reset();
    void sendDebug();

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{
        baseband_fs, this, baseband::Direction::Receive, /*auto_start*/ false};
    RSSIThread rssi_thread{};
};

#endif /*__PROC_ACARS_H__*/
