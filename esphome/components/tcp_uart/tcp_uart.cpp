#include "tcp_uart.h"

#include "esphome/components/socket/mbap.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::tcp_uart {

static const char *const TAG = "tcp_uart";

static constexpr uint32_t DROP_LOG_INTERVAL_MS = 5000;

void TcpUart::setup() {
  this->link_.begin(TAG);
#ifdef USE_SOCKET_TCP_LISTENER
  this->listener_.begin(TAG);
#endif
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(false);
  }
}

void TcpUart::dump_config() {
  ESP_LOGCONFIG(TAG,
                "TCP UART:\n"
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

void TcpUart::on_shutdown() {
  this->link_.close();
#ifdef USE_SOCKET_TCP_LISTENER
  this->listener_.close();
#endif
}

void TcpUart::sync_link_() {
  bool up = this->link_.connected();
  this->link_was_up_ = up;
  if (!up) {
    this->rx_start_ = this->rx_end_ = 0;
  } else {
    this->note_io_();
  }
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(up);
  }
}

void TcpUart::read_socket_() {
  if (this->rx_start_ != 0) {
    this->rx_end_ -= this->rx_start_;
    std::memmove(this->rx_, this->rx_ + this->rx_start_, this->rx_end_);
    this->rx_start_ = 0;
  }
  size_t room = RX_BUFFER_SIZE - this->rx_end_;
  if (room == 0) {
    // Only a read that filled all free space gets here, so rx_pending_ is already set.
    return;
  }
  ssize_t count = this->link_.read(this->rx_ + this->rx_end_, room);
  if (count <= 0) {
    // A dropped link (-1) is cleaned up by sync_link_() on the next loop.
    if (count == 0) {
      this->rx_pending_ = false;
    }
    return;
  }
  this->rx_end_ += static_cast<uint16_t>(count);
  this->rx_pending_ = static_cast<size_t>(count) == room;
  this->note_io_();
}

bool TcpUart::maintain_link_() {
#ifdef USE_SOCKET_TCP_LISTENER
  if (this->server_) {
    // link_was_up_ holds the accept until the previous drop's edge has run,
    // so the sensor and the cleared RX buffer always see the disconnect.
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

void TcpUart::loop() {
  if (!this->maintain_link_()) {
    return;
  }
  if (this->rx_pending_ || this->link_.ready()) {
    this->read_socket_();
  }
  this->link_.flush_tx();
}

void TcpUart::write_array(const uint8_t *data, size_t len) {
  size_t queued = this->link_.queue(data, len);
  if (queued > 0) {
    this->note_io_();
  }
  if (queued < len) {
    uint32_t now = App.get_loop_component_start_time();
    if (this->last_drop_log_ms_ == 0 || now - this->last_drop_log_ms_ >= DROP_LOG_INTERVAL_MS) {
      ESP_LOGW(TAG, "%s, dropped %u bytes",
               this->link_.connected() ? LOG_STR_LITERAL("TX buffer full") : LOG_STR_LITERAL("Not connected"),
               static_cast<unsigned>(len - queued));
      this->last_drop_log_ms_ = now;
    }
  }
}

bool TcpUart::peek_byte(uint8_t *data) {
  if (this->rx_start_ == this->rx_end_) {
    return false;
  }
  *data = this->rx_[this->rx_start_];
  return true;
}

bool TcpUart::read_array(uint8_t *data, size_t len) {
  if (this->available() < len) {
    return false;
  }
  std::memcpy(data, this->rx_ + this->rx_start_, len);
  this->rx_start_ += static_cast<uint16_t>(len);
  return true;
}

uart::UARTFlushResult TcpUart::flush() {
  bool emptied = this->link_.flush_tx();
  if (!this->link_.connected()) {
    // A down link cannot have delivered anything, whether this flush dropped
    // it or an earlier loop() write did.
    return uart::UARTFlushResult::UART_FLUSH_RESULT_FAILED;
  }
  return emptied ? uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS : uart::UARTFlushResult::UART_FLUSH_RESULT_TIMEOUT;
}

void TcpUart::note_io_() { this->last_io_ms_ = App.get_loop_component_start_time(); }

void TcpUart::check_timeout_() {
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

#ifdef USE_TCP_UART_MODBUS
void TcpUartModbus::loop() {
  if (!this->maintain_link_()) {
    this->tcp_len_ = 0;
    this->tx_len_ = 0;
    return;
  }
  if (this->rx_pending_ || this->link_.ready()) {
    this->read_mbap_();
  }
  this->link_.flush_tx();
}

void TcpUartModbus::dump_config() {
  TcpUart::dump_config();
  ESP_LOGCONFIG(TAG, "  Protocol: %s", LOG_STR_LITERAL("modbus"));
}

void TcpUartModbus::write_array(const uint8_t *data, size_t len) { this->queue_rtu_(data, len); }

uart::UARTFlushResult TcpUartModbus::flush() {
  this->send_rtu_as_mbap_();
  return TcpUart::flush();
}

void TcpUartModbus::read_mbap_() {
  size_t room = sizeof(this->tcp_buf_) - this->tcp_len_;
  if (room == 0) {
    this->tcp_len_ = 0;
    return;
  }
  ssize_t count = this->link_.read(this->tcp_buf_ + this->tcp_len_, room);
  if (count <= 0) {
    if (count == 0) {
      this->rx_pending_ = false;
    }
    return;
  }
  this->tcp_len_ += static_cast<uint16_t>(count);
  this->rx_pending_ = static_cast<size_t>(count) == room;
  this->note_io_();
  while (this->tcp_len_ != 0) {
    socket::Mbap frame;
    size_t used = 0;
    switch (socket::take_mbap(this->tcp_buf_, this->tcp_len_, &frame, &used)) {
      case socket::MbapTake::NEED_MORE:
        return;
      case socket::MbapTake::BAD:
        std::memmove(this->tcp_buf_, this->tcp_buf_ + used, this->tcp_len_ - used);
        this->tcp_len_ -= static_cast<uint16_t>(used);
        continue;
      case socket::MbapTake::FRAME:
        break;
    }
    std::memmove(this->tcp_buf_, this->tcp_buf_ + used, this->tcp_len_ - used);
    this->tcp_len_ -= static_cast<uint16_t>(used);
    if (frame.txn != this->txn_) {
      ESP_LOGW(TAG, "Dropped transaction %u, expected %u", frame.txn, this->txn_);
      continue;
    }
    size_t rtu_len = frame.pdu_len + 3;
    if (this->rx_end_ + rtu_len > RX_BUFFER_SIZE) {
      continue;
    }
    this->rx_[this->rx_end_] = frame.unit;
    std::memcpy(this->rx_ + this->rx_end_ + 1, frame.pdu, frame.pdu_len);
    uint16_t crc = crc16(this->rx_ + this->rx_end_, static_cast<uint16_t>(frame.pdu_len + 1));
    this->rx_[this->rx_end_ + frame.pdu_len + 1] = crc & 0xFF;
    this->rx_[this->rx_end_ + frame.pdu_len + 2] = crc >> 8;
    this->rx_end_ += static_cast<uint16_t>(rtu_len);
  }
}

void TcpUartModbus::queue_rtu_(const uint8_t *data, size_t len) {
  size_t before = this->tx_len_;
  size_t room = sizeof(this->tx_) - this->tx_len_;
  size_t n = std::min(len, room);
  std::memcpy(this->tx_ + this->tx_len_, data, n);
  this->tx_len_ += static_cast<uint16_t>(n);
  if (n < len) {
    ESP_LOGW(TAG, "RTU frame too long, dropped");
    this->tx_len_ = 0;
    return;
  }
  // The modbus component writes one whole RTU frame and does not flush
  // unless a flow-control pin is set. A frame that arrives in that one
  // write carries its CRC already, so it can go out here.
  if (before == 0 && socket::rtu_crc_ok(this->tx_, this->tx_len_)) {
    this->send_rtu_as_mbap_();
  }
}

void TcpUartModbus::send_rtu_as_mbap_() {
  if (!socket::rtu_crc_ok(this->tx_, this->tx_len_) || !this->link_.connected()) {
    this->tx_len_ = 0;
    return;
  }
  this->txn_ = this->txn_ == 0xFFFF ? 1 : static_cast<uint16_t>(this->txn_ + 1);
  uint8_t frame[TcpUartModbus::TCP_FRAME_SIZE];
  size_t n = socket::write_mbap(frame, sizeof(frame), this->txn_, this->tx_[0], this->tx_ + 1, this->tx_len_ - 3);
  this->tx_len_ = 0;
  if (n == 0) {
    return;
  }
  this->link_.queue(frame, n);
  this->note_io_();
}
#endif

}  // namespace esphome::tcp_uart
