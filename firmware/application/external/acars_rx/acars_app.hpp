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

#ifndef __ACARS_APP_H__
#define __ACARS_APP_H__

#include "app_settings.hpp"
#include "radio_state.hpp"
#include "ui_widget.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "ui_rssi.hpp"
#include "log_file.hpp"

#include <cstdint>
#include <cstddef>

namespace ui::external_app::acars_rx {

// Decoded ACARS frame fields.
//
// Field layout, 7-bit deparity and CRC handling are reimplemented from the
// algorithm in libacars la_acars_parse() (Copyright (c) 2018-2023
// Tomasz Lemiech, https://github.com/szpajder/libacars). This is a clean
// reimplementation against fixed buffers, not a copy of that source.
//
// crc_ok reflects the ACARS CRC: reflected CRC-16/CCITT (Kermit, poly 0x8408,
// init 0x0000). Running it over the frame bytes (parity still set) followed by
// the two received CRC bytes yields 0 on a valid frame.
struct AcarsDecoded {
    bool crc_ok{false};       // CRC residue check passed
    bool valid{false};        // structural parse succeeded
    bool more{false};         // ETB -> more blocks follow; ETX -> final block
    bool downlink{false};     // block_id in '0'..'9' => air-to-ground
    char mode{' '};
    char ack{' '};
    char block_id{' '};
    char msg_num_seq{' '};    // downlink only
    std::string reg{};
    std::string label{};
    std::string msg_num{};    // downlink only
    std::string flight_id{};  // downlink only
    std::string txt{};
};

// Decode a raw ACARS frame as delivered by the baseband processor:
//   raw      = bytes from MODE .. ETX/ETB inclusive, parity bits still set
//   raw_len  = length of raw
//   crc_lo   = first  trailing CRC byte (LSB), captured by the proc
//   crc_hi   = second trailing CRC byte (MSB), captured by the proc
// On a structural error, returns with valid == false and txt set to the reason
// (crc_ok is still meaningful).
AcarsDecoded acars_decode(const uint8_t* raw, size_t raw_len, uint8_t crc_lo, uint8_t crc_hi);

// Format a decoded ACARS message for display or logging.
std::string acars_format(const AcarsDecoded& msg);

class ACARSLogger {
   public:
    Optional<File::Error> append(const std::filesystem::path& filename) {
        return log_file.append(filename);
    }
    void log_str(std::string msg);

   private:
    LogFile log_file{};
};

class ACARSAppView : public View {
   public:
    ACARSAppView(NavigationView& nav);
    ~ACARSAppView();

    void focus() override;

    std::string title() const override { return "ACARS"; };

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{
        131825000 /* frequency */,
        1750000 /* bandwidth */,
        2457600 /* sampling rate */
    };
    app_settings::SettingsManager settings_{
        "rx_acars", app_settings::Mode::RX};

    bool logging{false};
    uint32_t packet_counter{0};

    RFAmpField field_rf_amp{
        {13 * 8, UI_POS_Y(0)}};
    LNAGainField field_lna{
        {15 * 8, UI_POS_Y(0)}};
    VGAGainField field_vga{
        {18 * 8, UI_POS_Y(0)}};
    RSSI rssi{
        {UI_POS_X(21), 0, UI_POS_WIDTH_REMAINING(24), 4}};
    Channel channel{
        {UI_POS_X(21), 5, UI_POS_WIDTH_REMAINING(24), 4}};

    RxFrequencyField field_frequency{
        {UI_POS_X(0), 0 * 8},
        nav_};
    Checkbox check_log{
        {16 * 8, 1 * 16},
        3,
        "LOG",
        true};

    Console console{
        {0, 3 * 16, screen_width, 256}};

    AudioVolumeField field_volume{
        {screen_width - 2 * 8, 1 * 16}};

    std::unique_ptr<ACARSLogger> logger{};

    void on_packet(const ACARSPacketMessage* packet);

    MessageHandlerRegistration message_handler_packet{
        Message::ID::ACARSPacket,
        [this](Message* const p) {
            const auto message = static_cast<const ACARSPacketMessage*>(p);
            this->on_packet(message);
        }};
};

}  // namespace ui::external_app::acars_rx

#endif /*__ACARS_APP_H__*/
