#include "uart_channel.h"
#if defined(USE_NOISE_UART)

#include "esphome/core/helpers.h"

#include <algorithm>
#include <cstring>

namespace esphome::noise {

// Same key, two pipes. The prologue is what keeps a raw peer from joining a Modbus peer.
static constexpr char PROLOGUE_RAW[] = "NoiseUARTInit";
static constexpr char PROLOGUE_MODBUS[] = "NoiseUARTModbusInit";

size_t UartChannel::plain_budget(size_t tx_free) const {
  if (!this->ready_ || tx_free <= FRAME_HEADER_SIZE + MAC_SIZE) {
    return 0;
  }
  return std::min(MAX_PLAIN, tx_free - FRAME_HEADER_SIZE - MAC_SIZE);
}

void UartChannel::reset() {
  if (this->send_ != nullptr) {
    noise_cipherstate_free(this->send_);
    this->send_ = nullptr;
  }
  if (this->recv_ != nullptr) {
    noise_cipherstate_free(this->recv_);
    this->recv_ = nullptr;
  }
  this->hs_.abort();
  this->active_ = nullptr;
  this->rx_have_ = 0;
  this->rx_need_ = 0;
  this->tx_len_ = 0;
  this->tx_off_ = 0;
  this->plain_len_ = 0;
  this->plain_off_ = 0;
  this->started_ = false;
  this->ready_ = false;
  this->failed_ = false;
}

bool UartChannel::start_() {
  const char *text = PROLOGUE_RAW;
  size_t len = sizeof(PROLOGUE_RAW) - 1;
  if (this->modbus_) {
    text = PROLOGUE_MODBUS;
    len = sizeof(PROLOGUE_MODBUS) - 1;
  }
  uint8_t prologue[sizeof(PROLOGUE_MODBUS) - 1];
  progmem_memcpy(prologue, text, len);
  this->active_ = &this->hs_;
  int err = this->initiator_ ? this->hs_.init(this->ctx_, prologue, len)
                             : this->hs_.NoiseResponderHandshake::init(this->ctx_, prologue, len);
  if (err != 0) {
    this->failed_ = true;
    return false;
  }
  this->started_ = true;
  return true;
}

bool UartChannel::flush_tx_(socket::TcpClientLink &link) {
  if (this->tx_off_ >= this->tx_len_) {
    return true;
  }
  size_t n = link.queue(this->tx_ + this->tx_off_, this->tx_len_ - this->tx_off_);
  this->tx_off_ += static_cast<uint16_t>(n);
  if (this->tx_off_ >= this->tx_len_) {
    this->tx_len_ = 0;
    this->tx_off_ = 0;
  }
  return true;
}

bool UartChannel::pull_frame_(socket::TcpClientLink &link, size_t max_payload) {
  if (this->rx_need_ == 0) {
    this->rx_need_ = FRAME_HEADER_SIZE;
  }
  while (this->rx_have_ < this->rx_need_) {
    ssize_t n = link.read(this->rx_ + this->rx_have_, this->rx_need_ - this->rx_have_);
    if (n < 0) {
      this->failed_ = true;
      return false;
    }
    if (n == 0) {
      return false;
    }
    this->rx_have_ += static_cast<uint16_t>(n);
    if (this->rx_have_ == FRAME_HEADER_SIZE && this->rx_need_ == FRAME_HEADER_SIZE) {
      size_t payload = (static_cast<size_t>(this->rx_[1]) << 8) | this->rx_[2];
      if (this->rx_[0] != FRAME_INDICATOR || payload == 0 || payload > max_payload) {
        this->failed_ = true;
        return false;
      }
      this->rx_need_ = static_cast<uint16_t>(FRAME_HEADER_SIZE + payload);
    }
  }
  return true;
}

bool UartChannel::on_handshake_frame_() {
  size_t payload = this->rx_need_ - FRAME_HEADER_SIZE;
  if (this->rx_[FRAME_HEADER_SIZE] != HANDSHAKE_STATUS_OK) {
    this->failed_ = true;
    return false;
  }
  int err = this->active_->read_message(this->rx_ + FRAME_HEADER_SIZE + 1, payload - 1);
  this->rx_have_ = 0;
  this->rx_need_ = 0;
  if (err != 0) {
    this->failed_ = true;
    return false;
  }
  return true;
}

bool UartChannel::finish_() {
  int err = this->active_->split(this->send_, this->recv_);
  if (err != 0) {
    this->failed_ = true;
    return false;
  }
  this->ready_ = true;
  this->active_ = nullptr;
  return true;
}

bool UartChannel::service(socket::TcpClientLink &link) {
  if (this->failed_) {
    return false;
  }
  if (this->ready_) {
    return true;
  }
  if (!this->started_ && !this->start_()) {
    return false;
  }
  if (this->tx_len_ > this->tx_off_) {
    return this->flush_tx_(link);
  }
  switch (this->active_->action()) {
    case NoiseResponderHandshake::Action::ACTION_WRITE: {
      if (link.tx_free() < FRAME_HEADER_SIZE + 1 + MAX_HANDSHAKE_SIZE) {
        return true;
      }
      size_t msg_len = 0;
      int err = this->active_->write_message(this->tx_ + FRAME_HEADER_SIZE + 1, MAX_HANDSHAKE_SIZE, msg_len);
      if (err != 0) {
        this->failed_ = true;
        return false;
      }
      this->tx_[FRAME_HEADER_SIZE] = HANDSHAKE_STATUS_OK;
      write_frame_header(this->tx_, static_cast<uint16_t>(msg_len + 1));
      this->tx_len_ = static_cast<uint16_t>(FRAME_HEADER_SIZE + 1 + msg_len);
      this->tx_off_ = 0;
      return this->flush_tx_(link);
    }
    case NoiseResponderHandshake::Action::ACTION_READ:
      if (!this->pull_frame_(link, 1 + MAX_HANDSHAKE_SIZE)) {
        return !this->failed_;
      }
      return this->on_handshake_frame_();
    case NoiseResponderHandshake::Action::ACTION_SPLIT:
      return this->finish_();
    default:
      this->failed_ = true;
      return false;
  }
}

ssize_t UartChannel::read(socket::TcpClientLink &link, uint8_t *buf, size_t len) {
  if (this->failed_ || !this->ready_ || len == 0) {
    return this->failed_ ? -1 : 0;
  }
  if (this->plain_off_ >= this->plain_len_) {
    if (!this->pull_frame_(link, MAX_PLAIN + MAC_SIZE)) {
      return this->failed_ ? -1 : 0;
    }
    size_t payload = this->rx_need_ - FRAME_HEADER_SIZE;
    NoiseBuffer mbuf;
    noise_buffer_init(mbuf);
    noise_buffer_set_inout(mbuf, this->rx_ + FRAME_HEADER_SIZE, payload, payload);
    int err = noise_cipherstate_decrypt(this->recv_, &mbuf);
    if (err != 0) {
      this->rx_have_ = 0;
      this->rx_need_ = 0;
      this->failed_ = true;
      return -1;
    }
    this->plain_off_ = FRAME_HEADER_SIZE;
    this->plain_len_ = static_cast<uint16_t>(FRAME_HEADER_SIZE + mbuf.size);
    this->rx_have_ = 0;
    this->rx_need_ = 0;
  }
  size_t n = std::min(len, static_cast<size_t>(this->plain_len_ - this->plain_off_));
  std::memcpy(buf, this->rx_ + this->plain_off_, n);
  this->plain_off_ += static_cast<uint16_t>(n);
  if (this->plain_off_ >= this->plain_len_) {
    this->plain_off_ = 0;
    this->plain_len_ = 0;
  }
  return static_cast<ssize_t>(n);
}

bool UartChannel::encrypt_and_queue_(socket::TcpClientLink &link, const uint8_t *data, size_t len) {
  std::memcpy(this->tx_ + FRAME_HEADER_SIZE, data, len);
  NoiseBuffer mbuf;
  noise_buffer_init(mbuf);
  noise_buffer_set_inout(mbuf, this->tx_ + FRAME_HEADER_SIZE, len, len + MAC_SIZE);
  int err = noise_cipherstate_encrypt(this->send_, &mbuf);
  if (err != 0) {
    return false;
  }
  write_frame_header(this->tx_, static_cast<uint16_t>(mbuf.size));
  size_t frame = FRAME_HEADER_SIZE + mbuf.size;
  return link.queue(this->tx_, frame) == frame;
}

size_t UartChannel::write(socket::TcpClientLink &link, const uint8_t *data, size_t len) {
  if (this->failed_ || !this->ready_ || len == 0) {
    return 0;
  }
  size_t accepted = 0;
  while (accepted < len) {
    size_t room = this->plain_budget(link.tx_free());
    if (room == 0) {
      break;
    }
    size_t n = std::min(room, len - accepted);
    if (!this->encrypt_and_queue_(link, data + accepted, n)) {
      this->failed_ = true;
      break;
    }
    accepted += n;
  }
  return accepted;
}

}  // namespace esphome::noise
#endif
