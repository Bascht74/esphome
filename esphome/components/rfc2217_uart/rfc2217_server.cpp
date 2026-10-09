#include "esphome/core/defines.h"
#ifdef USE_RFC2217_UART_SERVER

#include "rfc2217_uart.h"

#include "esphome/components/tcp_uart/tcp_uart.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::rfc2217_uart {

ESPHOME_LOG_TAG(TAG, "rfc2217_uart");

// Keeps the pacing product in 32 bits up to about 10 Mbaud.
static constexpr uint32_t MAX_PACE_SPAN_MS = 4000;
// A rate the UART cannot reach fails it until reboot on ESP-IDF, so the server takes only these.
static constexpr uint32_t MIN_BAUD_RATE = 300;
static constexpr uint32_t MAX_BAUD_RATE = 5000000;

void Rfc2217Server::dump_config() {
  ESP_LOGCONFIG(TAG, "RFC 2217 UART:\n"
                     "  Role: server");
}

void Rfc2217Server::on_link(bool up) {
  this->to_serial_len_ = 0;
  if (up) {
    // The driver kept whatever arrived while the link was down.
    this->discard_serial_();
  }
}

void Rfc2217Server::loop() {
  if (!this->sync_link_()) {
    return;
  }
  this->read_tcp_();
  this->write_serial_();
  this->update_flow_(this->to_serial_len_, TO_SERIAL_SIZE);
  this->read_serial_();
  this->flush_tcp_();
}

void Rfc2217Server::deliver(const uint8_t *data, size_t len) {
  // read_tcp_() takes no more than payload_room().
  std::memcpy(this->to_serial_ + this->to_serial_len_, data, len);
  this->to_serial_len_ = static_cast<uint16_t>(this->to_serial_len_ + len);
}

size_t Rfc2217Server::serial_room_() {
  // Same pacing as uart_tcp; to move into a uart helper.
  size_t room = this->parent_->available_for_write();
  if (room != SIZE_MAX) {
    return room;
  }
  // Capacity unknown on this platform; pace to the UART time since the last write,
  // at most one loop interval and 4 s, so a pass woken early writes little.
  uint32_t span =
      std::min({App.get_loop_component_start_time() - this->last_write_ms_, App.get_loop_interval(), MAX_PACE_SPAN_MS});
  // 10 bits per byte on the line.
  uint32_t paced = this->parent_->get_baud_rate() / 10 * span / 1000;
  return std::max<size_t>(1, paced);
}

void Rfc2217Server::write_serial_() {
  if (this->to_serial_len_ == 0) {
    return;
  }
  size_t n = std::min<size_t>(this->serial_room_(), this->to_serial_len_);
  if (n == 0) {
    return;
  }
  this->write_array(this->to_serial_, n);
  this->last_write_ms_ = App.get_loop_component_start_time();
  this->to_serial_len_ = static_cast<uint16_t>(this->to_serial_len_ - n);
  std::memmove(this->to_serial_, this->to_serial_ + n, this->to_serial_len_);
}

void Rfc2217Server::read_serial_() {
  if (this->peer_suspended_) {
    return;
  }
  // Every byte may double.
  size_t want = std::min({this->available(), this->tcp_->available_for_write() / 2, READ_CHUNK});
  uint8_t raw[READ_CHUNK];
  if (want == 0 || !this->read_array(raw, want)) {
    return;
  }
  uint8_t out[2 * READ_CHUNK];
  size_t used = 0;
  this->write_tcp_(out, telnet_escape(raw, want, out, sizeof(out), &used));
}

void Rfc2217Server::discard_serial_() {
  // Drain exactly what is buffered; later bytes are live.
  uint8_t dump[32];
  size_t left = this->available();
  while (left != 0) {
    size_t n = std::min(left, sizeof(dump));
    if (!this->read_array(dump, n)) {
      return;
    }
    left -= n;
  }
}

bool Rfc2217Server::load_serial() {
#if defined(USE_ESP8266) || defined(USE_ESP32)
  this->parent_->load_settings(false);
  return true;
#else
  return false;
#endif
}

void Rfc2217Server::apply_settings_(uint32_t baud_rate, uint8_t data_bits, uart::UARTParityOptions parity,
                                    uint8_t stop_bits) {
  uart::UARTComponent *serial = this->parent_;
  const uint32_t old_baud_rate = serial->get_baud_rate();
  const uint8_t old_data_bits = serial->get_data_bits();
  const uart::UARTParityOptions old_parity = serial->get_parity();
  const uint8_t old_stop_bits = serial->get_stop_bits();
  if (baud_rate == old_baud_rate && data_bits == old_data_bits && parity == old_parity && stop_bits == old_stop_bits) {
    return;
  }
  serial->set_baud_rate(baud_rate);
  serial->set_data_bits(data_bits);
  serial->set_parity(parity);
  serial->set_stop_bits(stop_bits);
  if (this->load_serial()) {
    ESP_LOGD(TAG, "Line set to %" PRIu32 " baud, %u data bits, parity %s, %u stop bits", baud_rate, data_bits,
             LOG_STR_ARG(uart::parity_to_str(parity)), stop_bits);
    return;
  }
  // The answer then reports what stays in use.
  serial->set_baud_rate(old_baud_rate);
  serial->set_data_bits(old_data_bits);
  serial->set_parity(old_parity);
  serial->set_stop_bits(old_stop_bits);
}

void Rfc2217Server::answer_(uint8_t code, uint8_t value) { this->send_command_(code, &value, 1); }

void Rfc2217Server::on_command(uint8_t code, const uint8_t *value, size_t len) {
  uart::UARTComponent *serial = this->parent_;
  const uint32_t baud_rate = serial->get_baud_rate();
  const uint8_t data_bits = serial->get_data_bits();
  const uart::UARTParityOptions parity = serial->get_parity();
  const uint8_t stop_bits = serial->get_stop_bits();
  // [RFC 2217] Every answer carries the value in use, so a value this UART cannot take is answered with the old one.
  switch (code) {
    case COM_FLOWCONTROL_SUSPEND:
      this->peer_suspended_ = true;
      return;
    case COM_FLOWCONTROL_RESUME:
      this->peer_suspended_ = false;
      return;
    default:
      break;
  }
  if (len == 0) {
    return;
  }
  const uint8_t v = value[0];
  switch (code) {
    case COM_SET_BAUDRATE: {
      if (len < 4) {
        return;
      }
      // 0 asks for the current value.
      uint32_t asked = encode_uint32(value[0], value[1], value[2], value[3]);
      if (asked >= MIN_BAUD_RATE && asked <= MAX_BAUD_RATE) {
        this->apply_settings_(asked, data_bits, parity, stop_bits);
      }
      const uint32_t now = serial->get_baud_rate();
      const uint8_t answer[4] = {static_cast<uint8_t>(now >> 24), static_cast<uint8_t>(now >> 16),
                                 static_cast<uint8_t>(now >> 8), static_cast<uint8_t>(now)};
      this->send_command_(COM_SET_BAUDRATE, answer, sizeof(answer));
      return;
    }
    case COM_SET_DATASIZE:
      if (v >= 5 && v <= 8) {
        this->apply_settings_(baud_rate, v, parity, stop_bits);
      }
      this->answer_(COM_SET_DATASIZE, serial->get_data_bits());
      return;
    case COM_SET_PARITY:
      // MARK and SPACE have no UARTParityOptions value.
      if (v == PARITY_NONE) {
        this->apply_settings_(baud_rate, data_bits, uart::UART_CONFIG_PARITY_NONE, stop_bits);
      } else if (v == PARITY_ODD) {
        this->apply_settings_(baud_rate, data_bits, uart::UART_CONFIG_PARITY_ODD, stop_bits);
      } else if (v == PARITY_EVEN) {
        this->apply_settings_(baud_rate, data_bits, uart::UART_CONFIG_PARITY_EVEN, stop_bits);
      }
      this->answer_(COM_SET_PARITY, to_rfc_parity(serial->get_parity()));
      return;
    case COM_SET_STOPSIZE:
      // 3 is 1.5 stop bits.
      if (v == 1 || v == 2) {
        this->apply_settings_(baud_rate, data_bits, parity, v);
      }
      this->answer_(COM_SET_STOPSIZE, serial->get_stop_bits());
      return;
    case COM_SET_CONTROL:
      if (v <= 3 || v == 17 || v == 19) {
        // Outbound flow control: the UART has none.
        this->answer_(COM_SET_CONTROL, CONTROL_NO_FLOW);
      } else if (v <= 6) {
        // BREAK is never sent.
        this->answer_(COM_SET_CONTROL, CONTROL_BREAK_OFF);
      } else if ((v >= 13 && v <= 16) || v == 18) {
        // Inbound flow control: the UART has none.
        this->answer_(COM_SET_CONTROL, CONTROL_NO_FLOW_IN);
      }
      // 7 to 12 are DTR and RTS: no such line, so no state to report.
      return;
    case COM_SET_LINESTATE_MASK:
    case COM_SET_MODEMSTATE_MASK:
      // No line or modem state is ever reported.
      this->answer_(code, 0);
      return;
    case COM_PURGE_DATA:
      if (v < 1 || v > 3) {
        return;
      }
      // 1 purges the bytes from the UART for the client, 2 those for the UART not yet written.
      if (v & 1) {
        this->discard_serial_();
      }
      if (v & 2) {
        this->to_serial_len_ = 0;
      }
      this->answer_(COM_PURGE_DATA, v);
      return;
    default:
      return;
  }
}

}  // namespace esphome::rfc2217_uart

#endif  // USE_RFC2217_UART_SERVER
