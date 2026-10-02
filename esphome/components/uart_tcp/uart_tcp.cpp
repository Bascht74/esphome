#include "uart_tcp.h"

#include "esphome/components/socket/mbap.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cstring>

namespace esphome::uart_tcp {

static const char *const TAG = "uart_tcp";

// Bytes per 16 ms loop pass at 10 bits per byte: baud / 10 / 62.5.
static constexpr uint32_t BAUD_PACE_DIVISOR = 625;

void UartTcp::setup() {
  this->link_.begin(TAG);
#ifdef USE_SOCKET_TCP_LISTENER
  this->listener_.begin(TAG);
#endif
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
}

void UartTcp::dump_config() {
  ESP_LOGCONFIG(TAG,
                "UART TCP:\n"
                "  %s: %s:%u\n"
                "  Reconnect Interval: %" PRIu32 "ms\n"
                "  Timeout: %" PRIu32 "ms",
                this->server_ ? LOG_STR_LITERAL("Listen") : LOG_STR_LITERAL("Host"),
                this->server_ ? LOG_STR_LITERAL("*") : this->link_.host(), this->link_.port(),
                this->link_.reconnect_interval(), this->timeout_ms_);
#ifdef USE_SOCKET_TCP_LISTENER
  this->listener_.dump_config();
#endif
  LOG_BINARY_SENSOR("  ", "Connected", this->connected_sensor_);
}

void UartTcp::on_shutdown() {
  this->link_.close();
#ifdef USE_SOCKET_TCP_LISTENER
  this->listener_.close();
#endif
}

void UartTcp::sync_link_() {
  bool up = this->link_.connected();
  this->link_was_up_ = up;
  if (up) {
    // The driver kept whatever arrived while the link was down.
    this->discard_uart_();
    this->note_io_();
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(up);
  }
}

void UartTcp::read_socket_() {
  // A hardware write blocks until the driver takes every byte. Leave what does
  // not fit in the socket, so TCP flow control throttles the peer.
  size_t room = this->parent_->available_for_write();
  if (room == SIZE_MAX) {
    // Capacity unknown on this platform; pace to one loop pass of UART time
    // (16 ms at 10 bits per byte) so a blocking write stays short.
    room = std::max<size_t>(1, this->parent_->get_baud_rate() / BAUD_PACE_DIVISOR);
  }
  if (room == 0) {
    this->rx_pending_ = true;
    return;
  }
  uint8_t tmp[READ_CHUNK];
  size_t want = std::min(room, sizeof(tmp));
  ssize_t count = this->link_.read(tmp, want);
  if (count <= 0) {
    // A dropped link (-1) is cleaned up by sync_link_() on the next loop.
    if (count == 0) {
      this->rx_pending_ = false;
    }
    return;
  }
  this->rx_pending_ = static_cast<size_t>(count) == want;
  this->write_array(tmp, static_cast<size_t>(count));
  this->note_io_();
}

void UartTcp::discard_uart_() {
  // Drain exactly what was buffered while the link was down; later bytes are live.
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

void UartTcp::read_uart_() {
  size_t want = std::min<size_t>(this->available(), this->link_.tx_free());
  if (want != 0 && this->read_array(this->link_.tx_tail(), want)) {
    this->link_.tx_commit(want);
    this->note_io_();
  }
}

bool UartTcp::maintain_link_() {
#ifdef USE_SOCKET_TCP_LISTENER
  if (this->server_) {
    // link_was_up_ holds the accept until the previous drop's edge has run,
    // so the sensor and the stale UART discard always see the disconnect.
    this->listener_.poll(this->link_, !this->link_was_up_);
  } else {
    this->link_.poll();
  }
#else
  this->link_.poll();
#endif
  if (this->link_.connected() != this->link_was_up_) {
    this->sync_link_();
  }
  if (!this->link_was_up_) {
    return false;
  }
  this->check_timeout_();
  return this->link_.connected();
}

void UartTcp::loop() {
  if (!this->maintain_link_()) {
    return;
  }
  if (this->rx_pending_ || this->link_.ready()) {
    this->read_socket_();
  }
  // UART bytes picked up here go out in the same pass.
  this->read_uart_();
  this->link_.flush_tx();
}

void UartTcp::note_io_() { this->last_io_ms_ = App.get_loop_component_start_time(); }

void UartTcp::check_timeout_() {
  if (this->timeout_ms_ == 0 || !this->link_was_up_) {
    return;
  }
  if (App.get_loop_component_start_time() - this->last_io_ms_ < this->timeout_ms_) {
    return;
  }
  ESP_LOGW(TAG, "Timeout, closing");
  this->link_.close();
  this->link_.note_attempt();
}

#ifdef USE_UART_TCP_MODBUS
void UartTcpModbus::loop() {
  if (!this->maintain_link_()) {
    this->tcp_len_ = 0;
    this->uart_len_ = 0;
    this->rtu_tx_len_ = 0;
    this->rtu_tx_off_ = 0;
    this->wait_uart_ = false;
    this->wait_tcp_ = false;
    this->arm_wait_uart_ = false;
    return;
  }
  this->pump_modbus_();
  this->link_.flush_tx();
}

void UartTcpModbus::dump_config() {
  UartTcp::dump_config();
  ESP_LOGCONFIG(TAG,
                "  Protocol: %s\n"
                "  Send Wait Time: %" PRIu32 "ms",
                LOG_STR_LITERAL("modbus"), this->send_wait_ms_);
}

uint32_t UartTcpModbus::frame_gap_us_() const {
  uint32_t baud = std::max<uint32_t>(1, this->parent_->get_baud_rate());
  // Above 19200 baud Modbus RTU fixes the gap at 1750 us. Below that it is
  // 3.5 character times, and a character here is 11 bits.
  if (baud > 19200) {
    return 1750;
  }
  return 38500000u / baud + 1;
}

void UartTcpModbus::read_tcp_buf_() {
  size_t room = sizeof(this->tcp_buf_) - this->tcp_len_;
  if (room == 0 || !this->link_.ready()) {
    return;
  }
  ssize_t count = this->link_.read(this->tcp_buf_ + this->tcp_len_, std::min(room, READ_CHUNK));
  if (count > 0) {
    this->tcp_len_ += static_cast<uint16_t>(count);
    this->note_io_();
  }
}

void UartTcpModbus::pull_uart_buf_() {
  size_t room = sizeof(this->uart_buf_) - this->uart_len_;
  size_t want = std::min(room, this->available());
  if (want == 0) {
    return;
  }
  if (this->read_array(this->uart_buf_ + this->uart_len_, want)) {
    this->uart_len_ += static_cast<uint16_t>(want);
    this->last_uart_us_ = micros();
    this->note_io_();
  }
}

bool UartTcpModbus::take_rtu_(uint8_t *pdu, size_t *pdu_len, uint8_t *unit) {
  if (this->uart_len_ < 4 || micros() - this->last_uart_us_ < this->frame_gap_us_()) {
    return false;
  }
  if (!modbus::rtu_crc_ok(this->uart_buf_, this->uart_len_)) {
    ESP_LOGW(TAG, "RTU CRC mismatch");
    this->uart_len_ = 0;
    return false;
  }
  *unit = this->uart_buf_[0];
  *pdu_len = this->uart_len_ - 3;
  std::memcpy(pdu, this->uart_buf_ + 1, *pdu_len);
  this->uart_len_ = 0;
  return true;
}

bool UartTcpModbus::drain_rtu_() {
  if (this->rtu_tx_off_ >= this->rtu_tx_len_) {
    return true;
  }
  size_t room = this->parent_->available_for_write();
  if (room == SIZE_MAX) {
    // Same pace as the raw bridge: one loop of UART time, so the write cannot block.
    room = std::max<size_t>(1, this->parent_->get_baud_rate() / BAUD_PACE_DIVISOR);
  }
  size_t left = this->rtu_tx_len_ - this->rtu_tx_off_;
  size_t n = std::min(room, left);
  if (n == 0) {
    return false;
  }
  this->parent_->write_array(this->rtu_tx_ + this->rtu_tx_off_, n);
  this->rtu_tx_off_ += static_cast<uint16_t>(n);
  if (this->rtu_tx_off_ < this->rtu_tx_len_) {
    return false;
  }
  this->rtu_tx_len_ = 0;
  this->rtu_tx_off_ = 0;
  return true;
}

bool UartTcpModbus::write_rtu_(const uint8_t *pdu, size_t pdu_len, uint8_t unit) {
  if (pdu_len + 3 > sizeof(this->rtu_tx_) || this->rtu_tx_len_ != 0) {
    return false;
  }
  this->rtu_tx_[0] = unit;
  std::memcpy(this->rtu_tx_ + 1, pdu, pdu_len);
  size_t len = pdu_len + 1;
  uint16_t crc = crc16(this->rtu_tx_, static_cast<uint16_t>(len));
  this->rtu_tx_[len++] = crc & 0xFF;
  this->rtu_tx_[len++] = crc >> 8;
  this->rtu_tx_len_ = static_cast<uint16_t>(len);
  this->rtu_tx_off_ = 0;
  return this->drain_rtu_();
}

void UartTcpModbus::send_mbap_(uint16_t txn, uint8_t unit, const uint8_t *pdu, size_t pdu_len) {
  uint8_t frame[UartTcpModbus::TCP_FRAME_SIZE];
  size_t n = modbus::write_mbap(frame, sizeof(frame), txn, unit, pdu, pdu_len);
  if (n == 0 || this->link_.tx_free() < n) {
    ESP_LOGW(TAG, "TX buffer full, dropped the Modbus frame");
    return;
  }
  this->link_.queue(frame, n);
  this->note_io_();
}

void UartTcpModbus::pump_modbus_() {
  uint8_t pdu[253];
  size_t pdu_len = 0;
  uint8_t unit = 0;
  uint32_t now = App.get_loop_component_start_time();
  if (this->rtu_tx_len_ != 0 && !this->drain_rtu_()) {
    if (this->arm_wait_uart_ && now - this->wait_started_ms_ >= this->send_wait_ms_) {
      ESP_LOGW(TAG, "Modbus response timeout");
      uint8_t exc[2] = {static_cast<uint8_t>(this->pending_function_ | 0x80), 0x0B};
      this->send_mbap_(this->txn_, this->pending_unit_, exc, sizeof(exc));
      this->rtu_tx_len_ = 0;
      this->rtu_tx_off_ = 0;
      this->arm_wait_uart_ = false;
    }
    return;
  }
  if (this->arm_wait_uart_) {
    this->arm_wait_uart_ = false;
    this->wait_uart_ = true;
    this->last_uart_us_ = micros();
  }
  if (this->wait_uart_ || this->wait_tcp_) {
    if (now - this->wait_started_ms_ >= this->send_wait_ms_) {
      ESP_LOGW(TAG, "Modbus response timeout");
      if (this->wait_uart_) {
        uint8_t exc[2] = {static_cast<uint8_t>(this->pending_function_ | 0x80), 0x0B};
        this->send_mbap_(this->txn_, this->pending_unit_, exc, sizeof(exc));
      }
      this->uart_len_ = 0;
      this->tcp_len_ = 0;
      this->rtu_tx_len_ = 0;
      this->rtu_tx_off_ = 0;
      this->wait_uart_ = false;
      this->wait_tcp_ = false;
      this->arm_wait_uart_ = false;
      return;
    }
  }
  if (this->wait_uart_) {
    this->pull_uart_buf_();
    if (this->take_rtu_(pdu, &pdu_len, &unit)) {
      this->send_mbap_(this->txn_, unit, pdu, pdu_len);
      this->wait_uart_ = false;
    }
    return;
  }
  if (this->wait_tcp_) {
    this->read_tcp_buf_();
    while (this->tcp_len_ != 0) {
      modbus::Mbap frame;
      size_t used = 0;
      switch (modbus::take_mbap(this->tcp_buf_, this->tcp_len_, &frame, &used)) {
        case modbus::MbapTake::NEED_MORE:
          return;
        case modbus::MbapTake::BAD:
          std::memmove(this->tcp_buf_, this->tcp_buf_ + used, this->tcp_len_ - used);
          this->tcp_len_ -= static_cast<uint16_t>(used);
          continue;
        case modbus::MbapTake::FRAME:
          break;
      }
      std::memmove(this->tcp_buf_, this->tcp_buf_ + used, this->tcp_len_ - used);
      this->tcp_len_ -= static_cast<uint16_t>(used);
      if (frame.txn != this->txn_) {
        ESP_LOGW(TAG, "Dropped transaction %u, expected %u", frame.txn, this->txn_);
        continue;
      }
      this->write_rtu_(frame.pdu, frame.pdu_len, frame.unit);
      this->wait_tcp_ = false;
      return;
    }
    return;
  }
  if (this->server_) {
    this->read_tcp_buf_();
    modbus::Mbap frame;
    size_t used = 0;
    switch (modbus::take_mbap(this->tcp_buf_, this->tcp_len_, &frame, &used)) {
      case modbus::MbapTake::NEED_MORE:
        return;
      case modbus::MbapTake::BAD:
        if (this->tcp_len_ != 0) {
          std::memmove(this->tcp_buf_, this->tcp_buf_ + 1, this->tcp_len_ - 1);
          this->tcp_len_--;
        }
        return;
      case modbus::MbapTake::FRAME:
        break;
    }
    std::memmove(this->tcp_buf_, this->tcp_buf_ + used, this->tcp_len_ - used);
    this->tcp_len_ -= static_cast<uint16_t>(used);
    if (frame.pdu_len == 0) {
      return;
    }
    this->txn_ = frame.txn;
    this->pending_unit_ = frame.unit;
    this->pending_function_ = frame.pdu[0];
    this->uart_len_ = 0;
    this->wait_started_ms_ = now;
    if (!this->write_rtu_(frame.pdu, frame.pdu_len, frame.unit)) {
      this->arm_wait_uart_ = true;
      return;
    }
    this->wait_uart_ = true;
    this->last_uart_us_ = micros();
    return;
  }
  this->pull_uart_buf_();
  if (!this->take_rtu_(pdu, &pdu_len, &unit) || pdu_len == 0) {
    return;
  }
  this->txn_ = this->txn_ == 0xFFFF ? 1 : static_cast<uint16_t>(this->txn_ + 1);
  this->pending_function_ = pdu[0];
  this->send_mbap_(this->txn_, unit, pdu, pdu_len);
  this->wait_tcp_ = true;
  this->wait_started_ms_ = now;
}
#endif

}  // namespace esphome::uart_tcp
