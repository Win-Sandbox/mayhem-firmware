/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
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

#include "baseband_api.hpp"
#include "portapack_persistent_memory.hpp"
#include "file_path.hpp"
#include "audio.hpp"

#include "acars_app.hpp"
using namespace portapack;

#include "string_format.hpp"
#include "utility.hpp"

#include <cstdint>
#include <cstddef>

namespace ui::external_app::acars_rx {

// ACARS frame as delivered by proc_acars (SOH already consumed, parity bits
// still present, ETX/ETB kept as the last byte, the 2 CRC bytes carried
// separately in packet->crc):
//   [0]        Mode
//   [1..7]     Aircraft registration
//   [8]        Technical acknowledge
//   [9..10]    Label
//   [11]       Block ID  ('0'..'9' => downlink / air-to-ground)
//   [12]       STX
//   downlink:  [13..15] msg number, [16] seq char, [17..22] flight ID, then text
//   uplink:    text directly (may be empty)
//   [last]     ETX (0x03, final) or ETB (0x17, more blocks follow)
//
// Field layout, parity stripping and CRC check follow the algorithm in
// libacars la_acars_parse() (Tomasz Lemiech). Reimplemented here against fixed
// buffers, without goto, with explicit bounds checks.

// Reflected CRC-16/CCITT (Kermit): poly 0x8408 (reverse of 0x1021), init 0,
// no final XOR. Feeding the frame bytes followed by the received CRC bytes
// yields 0 on a valid ACARS frame (the CRC has residue 0x0000).
static uint16_t acars_crc16(const uint8_t* data, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(crc ^ data[i]);
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408)
                            : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

AcarsDecoded acars_decode(const uint8_t* raw, size_t raw_len, uint8_t crc_lo, uint8_t crc_hi) {
    AcarsDecoded r;

    // CRC over the on-air bytes (parity intact) followed by the two received
    // CRC bytes. Result 0 means the frame is intact.
    uint16_t crc = acars_crc16(raw, raw_len, 0x0000);
    const uint8_t crc_bytes[2] = {crc_lo, crc_hi};
    r.crc_ok = (acars_crc16(crc_bytes, 2, crc) == 0);

    // Smallest legal frame: mode + reg(7) + ack + label(2) + block_id + ETX/ETB.
    if (raw_len < 13) {
        r.txt = "frame too short (" + std::to_string(raw_len) + ")";
        return r;
    }

    // Strip parity to recover 7-bit ASCII on a local copy.
    std::string buf(raw_len, '\0');
    for (size_t i = 0; i < raw_len; ++i)
        buf[i] = static_cast<char>(raw[i] & 0x7f);

    // Trailing ETX/ETB marks the end of the text block.
    const char end = buf[raw_len - 1];
    if (end == 0x03)
        r.more = false;
    else if (end == 0x17)
        r.more = true;
    else {
        r.txt = "missing ETX/ETB";
        return r;
    }
    const size_t len = raw_len - 1;  // length excluding ETX/ETB

    size_t p = 0;
    r.mode = buf[p++];
    r.reg = buf.substr(p, 7);
    p += 7;
    r.ack = buf[p++];
    if (r.ack == 0x15)
        r.ack = '!';  // NAK
    else if (r.ack == 0x06)
        r.ack = '^';  // ACK
    r.label = buf.substr(p, 2);
    p += 2;
    if (r.label.size() == 2 && r.label[1] == 0x7f)
        r.label[1] = 'd';
    r.block_id = buf[p++];
    if (r.block_id == 0)
        r.block_id = ' ';
    r.downlink = (r.block_id >= '0' && r.block_id <= '9');

    // An uplink with empty text has no STX and nothing more to parse.
    if (p >= len) {
        r.valid = true;
        return r;
    }
    if (buf[p] != 0x02) {  // STX expected after the preamble
        r.txt = "missing STX";
        return r;
    }
    p++;

    if (r.downlink) {
        // Downlink text is prefixed by msg number(3) + seq char(1) + flight ID(6).
        if (len - p < 10) {
            r.txt = "downlink text too short";
            return r;
        }
        r.msg_num = buf.substr(p, 3);
        p += 3;
        r.msg_num_seq = buf[p++];
        r.flight_id = buf.substr(p, 6);
        p += 6;
    }
    if (p < len)
        r.txt = buf.substr(p, len - p);
    r.valid = true;
    return r;
}

std::string acars_format(const AcarsDecoded& m) {
    std::string s = "ACARS";
    s += m.crc_ok ? "" : " (CRC ERR)";
    if (!m.valid) {
        // Structural parse failed; show the reason instead of empty fields.
        s += " - " + m.txt;
        return s;
    }
    s += "\nReg: " + m.reg;
    if (m.downlink)
        s += " Flight: " + m.flight_id;
    s += "\nMode: ";
    s += m.mode;
    s += " Label: " + m.label + " Blk: ";
    s += m.block_id;
    s += " Ack: ";
    s += m.ack;
    if (m.more)
        s += " More";
    if (m.downlink) {
        s += "\nMsgNum: " + m.msg_num;
        s += m.msg_num_seq;
    }
    if (!m.txt.empty())
        s += "\nMsg: " + m.txt;
    return s;
}

void ACARSLogger::log_str(std::string msg) {
    log_file.write_entry(msg);
}

ACARSAppView::ACARSAppView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&rssi,
                  &channel,
                  &field_rf_amp,
                  &field_lna,
                  &field_vga,
                  &field_frequency,
                  &field_volume,
                  &check_log,
                  &console});

    receiver_model.enable();

    check_log.set_value(logging);
    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
    };

    logger = std::make_unique<ACARSLogger>();
    if (logger)
        logger->append(logs_dir / u"ACARS.TXT");

    audio::set_rate(audio::Rate::Hz_24000);
    audio::output::start();
}

ACARSAppView::~ACARSAppView() {
    receiver_model.disable();
    baseband::shutdown();
}

void ACARSAppView::focus() {
    field_frequency.focus();
}

void ACARSAppView::on_packet(const ACARSPacketMessage* packet) {
    rtc::RTC datetime;
    rtc_time::now(datetime);
    std::string line = to_string_datetime(datetime, HMS) + ": ";

    if (packet->state == 255) {
        // Real payload: parse with the libacars-derived logic. The CRC bytes
        // were captured separately by the baseband proc.
        const AcarsDecoded decoded = acars_decode(
            reinterpret_cast<const uint8_t*>(packet->message),
            packet->msg_len, packet->crc[0], packet->crc[1]);
        line += acars_format(decoded);
    } else {
        // Debug/state packet from the baseband.
        line += "State " + to_string_dec_int(packet->state) +
                " last " + to_string_dec_uint(packet->message[0]);
    }

    console.writeln(line);
    if (logger && logging)
        logger->log_str(line);
}

}  // namespace ui::external_app::acars_rx
