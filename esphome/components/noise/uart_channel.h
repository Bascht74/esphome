#pragma once

#include "esphome/core/defines.h"

#if defined(USE_NOISE_UART)

#include "esphome/components/noise/noise_handshake.h"
#include "esphome/components/socket/tcp_client_link.h"
#include "esphome/core/log.h"

#include <cstddef>
#include <cstdint>

namespace esphome::noise {

/// One Noise_NNpsk0 pipe over an already connected TcpClientLink.
/// Handshake frames use the shared indicator/length header and a status byte.
/// After the split, each direction is one encrypted frame at a time.
/// Compiled only when a tcp_uart or uart_tcp instance sets encryption.
class UartChannel {
 public:
  void set_psk(const uint8_t *psk) { this->ctx_.set_psk(psk); }
  void set_initiator(bool initiator) { this->initiator_ = initiator; }
  /// Modbus speaks MBAP on this socket. A different prologue stops the same key
  /// from completing a handshake with a raw pipe.
  void set_modbus(bool modbus) { this->modbus_ = modbus; }

  bool ready() const { return this->ready_; }
  bool failed() const { return this->failed_; }
  /// A frame is half-read or plaintext is still waiting to be copied out.
  bool pending() const { return this->rx_have_ != 0 || this->plain_off_ < this->plain_len_; }
  /// How many plaintext bytes fit in the link's free TX space, one frame.
  size_t plain_budget(size_t tx_free) const;

  void reset();
  /// One handshake step. False means the link should be closed.
  bool service(socket::TcpClientLink &link);
  ssize_t read(socket::TcpClientLink &link, uint8_t *buf, size_t len);
  size_t write(socket::TcpClientLink &link, const uint8_t *data, size_t len);

 protected:
  bool start_();
  bool flush_tx_(socket::TcpClientLink &link);
  bool pull_frame_(socket::TcpClientLink &link, size_t max_payload);
  bool on_handshake_frame_();
  bool finish_();
  bool encrypt_and_queue_(socket::TcpClientLink &link, const uint8_t *data, size_t len);

  static constexpr size_t MAX_PLAIN = 512;
  static constexpr size_t FRAME_CAP = FRAME_HEADER_SIZE + MAX_PLAIN + MAC_SIZE;

  // The initiator type can also run the responder init. One object, one role.
  NoiseInitiatorHandshake hs_;
  NoiseResponderHandshake *active_{nullptr};
  NoiseCipherState *send_{nullptr};
  NoiseCipherState *recv_{nullptr};
  NoiseContext ctx_;
  uint16_t rx_have_{0};
  uint16_t rx_need_{0};
  uint16_t tx_len_{0};
  uint16_t tx_off_{0};
  uint16_t plain_len_{0};
  uint16_t plain_off_{0};
  bool initiator_{false};
  bool modbus_{false};
  bool started_{false};
  bool ready_{false};
  bool failed_{false};
  uint8_t rx_[FRAME_CAP]{};
  uint8_t tx_[FRAME_CAP]{};
};

/// Handshake only while it is not finished. Once ready(), this is an inline
/// check and does not call into the cipher code.
template<typename Self> bool uart_noise_ready(Self *self, const char *tag) {
  if (!self->maintain_link_()) {
    self->channel_.reset();
    return false;
  }
  if (self->channel_.ready()) {
    return true;
  }
  if (self->channel_.failed() || !self->channel_.service(self->link_)) {
    ESP_LOGW(tag, "Handshake failed");
    self->link_.close();
    self->link_.note_attempt();
    self->channel_.reset();
    return false;
  }
  self->link_.flush_tx();
  return self->channel_.ready();
}

}  // namespace esphome::noise

#endif
