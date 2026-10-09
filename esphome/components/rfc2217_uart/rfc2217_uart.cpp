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

static constexpr uint32_t DROP_LOG_INTERVAL_MS = 5000;
static constexpr size_t OPTION_BINARY_INDEX = 0;
static constexpr size_t OPTION_COM_PORT_INDEX = 1;

void Rfc2217Base::note_drop_(const LogString *message) {
  uint32_t now = App.get_loop_component_start_time();
  if (this->drop_log_ms_ != 0 && now - this->drop_log_ms_ < DROP_LOG_INTERVAL_MS) {
    return;
  }
  // A zero stamp would look like "never logged" on the next pass.
  this->drop_log_ms_ = now == 0 ? 1 : now;
  ESP_LOGW(TAG, "%s", LOG_STR_ARG(message));
}

bool Rfc2217Base::sync_link_() {
  bool up = this->tcp_->is_connected();
  if (up == this->link_was_up_) {
    return up;
  }
  this->link_was_up_ = up;
  this->decoder_.reset();
  for (size_t i = 0; i < 2; i++) {
    this->us_[i] = OptionState::NO;
    this->him_[i] = OptionState::NO;
  }
  this->peer_suspended_ = false;
  this->suspended_peer_ = false;
  this->on_link(up);
  if (up) {
    this->us_[OPTION_BINARY_INDEX] = OptionState::WANT_YES;
    this->him_[OPTION_BINARY_INDEX] = OptionState::WANT_YES;
    this->send_option_(TELNET_WILL, OPTION_BINARY);
    this->send_option_(TELNET_DO, OPTION_BINARY);
    // [RFC 2217] The client offers COM-PORT and the server accepts it; either may start.
    if (this->server_) {
      this->him_[OPTION_COM_PORT_INDEX] = OptionState::WANT_YES;
      this->send_option_(TELNET_DO, OPTION_COM_PORT);
    } else {
      this->us_[OPTION_COM_PORT_INDEX] = OptionState::WANT_YES;
      this->send_option_(TELNET_WILL, OPTION_COM_PORT);
    }
  }
  return up;
}

bool Rfc2217Base::com_port_() const {
  return (this->server_ ? this->him_[OPTION_COM_PORT_INDEX] : this->us_[OPTION_COM_PORT_INDEX]) == OptionState::YES;
}

void Rfc2217Base::write_tcp_(const uint8_t *data, size_t len) {
  this->tcp_->write_array(data, len);
  this->wrote_ = true;
}

void Rfc2217Base::flush_tcp_() {
  // Send now, not on tcp_uart's next pass.
  if (this->wrote_) {
    this->wrote_ = false;
    this->tcp_->flush();
  }
}

void Rfc2217Base::send_option_(uint8_t verb, uint8_t option) {
  const uint8_t command[3] = {TELNET_IAC, verb, option};
  this->write_tcp_(command, sizeof(command));
}

bool Rfc2217Base::send_command_(uint8_t code, const uint8_t *value, size_t len) {
  uint8_t command[COM_PORT_COMMAND_MAX];
  size_t n = write_com_port(command, this->server_ ? code + SERVER_OFFSET : code, value, len);
  if (this->tcp_->available_for_write() < n) {
    return false;
  }
  this->write_tcp_(command, n);
  return true;
}

void Rfc2217Base::on_option_(uint8_t verb, uint8_t option) {
  const bool positive = verb == TELNET_WILL || verb == TELNET_DO;
  // WILL and WONT are about the peer's side of the option, DO and DONT about this side.
  const bool his = verb == TELNET_WILL || verb == TELNET_WONT;
  const uint8_t yes = his ? TELNET_DO : TELNET_WILL;
  const uint8_t no = his ? TELNET_DONT : TELNET_WONT;
  size_t index;
  if (option == OPTION_BINARY) {
    index = OPTION_BINARY_INDEX;
  } else if (option == OPTION_COM_PORT) {
    index = OPTION_COM_PORT_INDEX;
  } else {
    // Refuse every other option; a refusal needs no answer.
    if (positive) {
      this->send_option_(no, option);
    }
    return;
  }
  OptionState &state = his ? this->him_[index] : this->us_[index];
  const bool asked = state == OptionState::WANT_YES;
  if (positive) {
    if (state == OptionState::NO) {
      this->send_option_(yes, option);
    }
    state = OptionState::YES;
    return;
  }
  if (state == OptionState::YES) {
    this->send_option_(no, option);
  }
  state = OptionState::NO;
  // The side RFC 2217 needs: the client's WILL, the server's DO.
  if (asked && index == OPTION_COM_PORT_INDEX && his == this->server_) {
    ESP_LOGW(TAG, "%s", LOG_STR_ARG(LOG_STR("Peer refused the COM-PORT option, the line settings are not exchanged")));
  }
}

void Rfc2217Base::read_tcp_() {
  uint8_t payload[READ_CHUNK];
  size_t n = 0;
  size_t room = std::min(this->payload_room(), sizeof(payload));
  while (n < room) {
    uint8_t byte;
    if (!this->tcp_->peek_byte(&byte)) {
      break;
    }
    // An answer goes out whole, so a command waits in the link until there is room for one.
    if ((byte == TELNET_IAC || !this->decoder_.idle()) && this->tcp_->available_for_write() < COM_PORT_COMMAND_MAX) {
      break;
    }
    this->tcp_->read_array(&byte, 1);
    switch (this->decoder_.feed(byte)) {
      case TelnetDecoder::Event::DATA:
        payload[n++] = this->decoder_.data();
        break;
      case TelnetDecoder::Event::OPTION:
        this->on_option_(this->decoder_.verb(), this->decoder_.option());
        break;
      case TelnetDecoder::Event::SUBNEGOTIATION: {
        // A command such as PURGE-DATA applies to the payload before it, so that goes first.
        if (n != 0) {
          this->deliver(payload, n);
          n = 0;
          room = std::min(this->payload_room(), sizeof(payload));
        }
        const uint8_t *sub = this->decoder_.sub();
        size_t len = this->decoder_.sub_len();
        if (len >= 2 && sub[0] == OPTION_COM_PORT) {
          this->on_command(sub[1], sub + 2, len - 2);
        }
        break;
      }
      case TelnetDecoder::Event::NONE:
        break;
    }
  }
  if (n != 0) {
    this->deliver(payload, n);
  }
}

void Rfc2217Base::update_flow_(size_t level, size_t capacity) {
  if (!this->com_port_()) {
    return;
  }
  if (!this->suspended_peer_ && level >= capacity * 3 / 4) {
    this->suspended_peer_ = this->send_command_(COM_FLOWCONTROL_SUSPEND, nullptr, 0);
  } else if (this->suspended_peer_ && level <= capacity / 4) {
    this->suspended_peer_ = !this->send_command_(COM_FLOWCONTROL_RESUME, nullptr, 0);
  }
}

}  // namespace esphome::rfc2217_uart
