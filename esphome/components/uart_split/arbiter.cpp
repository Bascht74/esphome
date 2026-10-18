#include "esphome/core/defines.h"
#ifdef USE_UART_SPLIT_ARBITER

#include "arbiter.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::uart_split {

ESPHOME_LOG_TAG(TAG, "uart_split");

static constexpr uint32_t US_PER_SEC = 1000000;
static constexpr uint32_t US_PER_MS = 1000;
static constexpr uint32_t MIN_FRAME_DELAY_US = 1750;
// Without a receive threshold the driver may hand bytes over in chunks, e.g. over USB.
static constexpr uint32_t CHUNKED_RX_DELAY_US = 50 * US_PER_MS;

static uint32_t bits_per_char(uart::UARTComponent *uart) {
  const uint32_t data_bits = uart->get_data_bits() != 0 ? uart->get_data_bits() : 8;
  const uint32_t stop_bits = uart->get_stop_bits() != 0 ? uart->get_stop_bits() : 1;
  return 1 + data_bits + (uart->get_parity() == uart::UART_CONFIG_PARITY_NONE ? 0 : 1) + stop_bits;
}

static uint32_t baud_rate(uart::UARTComponent *uart) { return std::max<uint32_t>(1u, uart->get_baud_rate()); }

// Signed, so a time still ahead reads as not reached. Every span here is far below the 35 minute wrap.
static bool elapsed(uint32_t now_us, uint32_t from_us, uint32_t span_us) {
  return static_cast<int32_t>(now_us - from_us) >= static_cast<int32_t>(span_us);
}

uint32_t char_time_us(uart::UARTComponent *uart) {
  const uint32_t baud = baud_rate(uart);
  return (bits_per_char(uart) * US_PER_SEC + baud - 1) / baud;
}

uint32_t frame_delay_us(uart::UARTComponent *uart) {
  return std::max(MIN_FRAME_DELAY_US, bits_per_char(uart) * (US_PER_SEC / 2 * 7) / baud_rate(uart) + 1);
}

uint32_t reply_gap_us(uart::UARTComponent *uart) {
  const size_t threshold = uart->get_rx_full_threshold();
  if (threshold == uart::UARTComponent::RX_FULL_THRESHOLD_UNSET) {
    return frame_delay_us(uart) + CHUNKED_RX_DELAY_US;
  }
  // A full FIFO is handed over at the threshold, the rest after the idle timeout.
  const uint64_t held_bits = static_cast<uint64_t>(threshold + uart->get_rx_timeout()) * bits_per_char(uart);
  return frame_delay_us(uart) + static_cast<uint32_t>(held_bits * US_PER_SEC / baud_rate(uart)) + 1;
}

void Arbiter::setup() {
  UartSplit *split = this->split_;
  size_t writers = 0;
  for (uint8_t i = 0; i < split->output_count_; i++) {
    writers += split->outputs_[i]->rx_only() ? 0 : 1;
  }
  this->requests_ = std::make_unique<Request[]>(split->output_count_);
  this->buffers_ = std::make_unique<uint8_t[]>(writers * TX_BUFFER_SIZE);
  uint8_t *next = this->buffers_.get();
  for (uint8_t i = 0; i < split->output_count_; i++) {
    if (!split->outputs_[i]->rx_only()) {
      this->requests_[i].data = next;
      next += TX_BUFFER_SIZE;
    }
  }
  uart::UARTComponent *parent = split->parent_;
  const uint32_t reply_gap = reply_gap_us(parent);
  // Writers write from their own loop(), so a request written over two passes stays one request.
  const uint32_t request_gap = std::max(reply_gap, 2 * App.get_loop_interval() * US_PER_MS);
  this->set_timing(char_time_us(parent), request_gap, reply_gap);
}

void Arbiter::set_timing(uint32_t char_us, uint32_t request_gap_us, uint32_t reply_gap_us) {
  this->char_us_ = char_us;
  this->request_gap_us_ = request_gap_us;
  this->reply_gap_us_ = reply_gap_us;
}

void Arbiter::loop(uint32_t now_us) {
  this->receive_(now_us);
  this->close_requests_(now_us);
  switch (this->line_) {
    case Line::SENT: {
      this->hand_queued_bytes_(now_us);
      const Request &req = this->requests_[this->owner_];
      // Not ACTIVE: the owner already writes its next request, so the last one is on the line.
      const bool on_line = req.state != RequestState::ACTIVE || req.sent == req.len;
      if (on_line && elapsed(now_us, this->tx_end_us_, this->send_wait_us_)) {
        ESP_LOGD(TAG, "No reply for output %u", this->owner_);
        this->enter_gap_(this->tx_end_us_ + this->send_wait_us_);
      }
      break;
    }
    case Line::RECEIVING:
      this->hand_queued_bytes_(now_us);
      if (elapsed(now_us, this->last_rx_us_, this->reply_gap_us_)) {
        this->enter_gap_(this->last_rx_us_ + this->reply_gap_us_);
      }
      break;
    case Line::GAP:
      if (elapsed(now_us, this->gap_from_us_, this->turnaround_us_)) {
        this->line_ = Line::IDLE;
      }
      break;
    case Line::IDLE:
      break;
  }
  if (this->line_ == Line::IDLE && this->queue_count_ != 0) {
    const uint8_t index = this->queue_[this->queue_head_];
    this->queue_head_ = (this->queue_head_ + 1) % MAX_OUTPUTS;
    this->queue_count_--;
    this->start_(index, now_us);
    this->hand_queued_bytes_(now_us);
  }
}

void Arbiter::receive_(uint32_t now_us) {
  UartSplit *split = this->split_;
  size_t waiting = split->parent_->available();
  if (waiting == 0) {
    return;
  }
  // A full output drops its bytes; nothing waits for it.
  size_t take = std::min(waiting, RX_BUFFER_SIZE);
  uint8_t buf[READ_CHUNK];
  while (take > 0) {
    size_t len = std::min(take, READ_CHUNK);
    if (!split->parent_->read_array(buf, len)) {
      break;
    }
    for (uint8_t i = 0; i < split->output_count_; i++) {
      UartSplitOutput *output = split->outputs_[i];
      if (output->rx_only() || i == this->owner_) {
        output->end_dropping_if_empty();
        split->push_(i, buf, len);
      }
    }
    take -= len;
  }
  this->last_rx_us_ = now_us;
  if (this->line_ == Line::SENT) {
    this->line_ = Line::RECEIVING;
  } else if (this->line_ != Line::RECEIVING) {
    // Unasked bytes: the line is busy, so the turnaround time starts again.
    this->line_ = Line::GAP;
    this->gap_from_us_ = now_us;
  }
}

void Arbiter::close_requests_(uint32_t now_us) {
  for (uint8_t i = 0; i < this->split_->output_count_; i++) {
    Request &req = this->requests_[i];
    if (req.dropping && elapsed(now_us, req.last_write_us, this->request_gap_us_)) {
      req.dropping = false;
    }
    if (req.state == RequestState::OPEN && elapsed(now_us, req.last_write_us, this->request_gap_us_)) {
      req.state = RequestState::QUEUED;
      this->queue_[(this->queue_head_ + this->queue_count_) % MAX_OUTPUTS] = i;
      this->queue_count_++;
    }
  }
}

void Arbiter::start_(uint8_t index, uint32_t now_us) {
  Request &req = this->requests_[index];
  req.state = RequestState::ACTIVE;
  req.sent = 0;
  this->owner_ = index;
  this->line_ = Line::SENT;
  this->tx_end_us_ = now_us;
}

void Arbiter::send_(const uint8_t *data, size_t len, uint32_t now_us) {
  this->split_->parent_->write_array(data, len);
  this->split_->mirror_tx(this->split_->outputs_[this->owner_], data, len);
  if (static_cast<int32_t>(now_us - this->tx_end_us_) > 0) {
    this->tx_end_us_ = now_us;
  }
  this->tx_end_us_ += static_cast<uint32_t>(len) * this->char_us_;
}

void Arbiter::hand_queued_bytes_(uint32_t now_us) {
  Request &req = this->requests_[this->owner_];
  // An owner that is OPEN already writes its next request while the reply comes in.
  if (req.state != RequestState::ACTIVE || req.sent >= req.len) {
    return;
  }
  // A hardware write blocks while its TX buffer is full, so only what fits now.
  size_t len = std::min<size_t>(req.len - req.sent, this->split_->parent_->available_for_write());
  if (len == 0) {
    return;
  }
  this->send_(req.data + req.sent, len, now_us);
  req.sent += len;
}

void Arbiter::enter_gap_(uint32_t from_us) {
  Request &req = this->requests_[this->owner_];
  if (req.state == RequestState::ACTIVE) {
    req.state = RequestState::FREE;
    req.len = 0;
    req.sent = 0;
  }
  this->owner_ = NO_OWNER;
  this->line_ = Line::GAP;
  this->gap_from_us_ = from_us;
}

void Arbiter::write(uint8_t index, const uint8_t *data, size_t len, uint32_t now_us) {
  Request &req = this->requests_[index];
  const bool continues = !elapsed(now_us, req.last_write_us, this->request_gap_us_);
  req.last_write_us = now_us;
  if (req.dropping) {
    return;
  }
  const bool handed = req.state == RequestState::ACTIVE && req.sent == req.len;
  if (handed && this->line_ == Line::SENT && continues) {
    // More of the request on the line, e.g. its second TCP segment.
    this->send_(data, len, now_us);
    return;
  }
  if (req.state == RequestState::FREE && this->line_ == Line::IDLE && this->queue_count_ == 0) {
    this->start_(index, now_us);
    this->send_(data, len, now_us);
    return;
  }
  if (handed || req.state == RequestState::FREE) {
    // A later write of the owner is its next request, and it waits like any other.
    req.state = RequestState::OPEN;
    req.len = 0;
    req.sent = 0;
  }
  if (req.state != RequestState::OPEN || len > TX_BUFFER_SIZE - req.len) {
    this->drop_(index);
    return;
  }
  std::memcpy(req.data + req.len, data, len);
  req.len += len;
}

void Arbiter::drop_(uint8_t index) {
  Request &req = this->requests_[index];
  if (req.state == RequestState::OPEN) {
    req.state = RequestState::FREE;
    req.len = 0;
  }
  req.dropping = true;
  ESP_LOGW(TAG, "Output %u: request dropped; each output can have one request of up to %u bytes waiting", index,
           static_cast<unsigned>(TX_BUFFER_SIZE));
}

size_t Arbiter::available_for_write(uint8_t index) const {
  const Request &req = this->requests_[index];
  if (req.dropping) {
    return 0;
  }
  switch (req.state) {
    case RequestState::FREE:
      if (this->line_ == Line::IDLE && this->queue_count_ == 0) {
        return this->split_->parent_->available_for_write();
      }
      return TX_BUFFER_SIZE;
    case RequestState::OPEN:
      return TX_BUFFER_SIZE - req.len;
    case RequestState::ACTIVE:
      if (req.sent < req.len) {
        return 0;
      }
      return this->line_ == Line::SENT ? this->split_->parent_->available_for_write() : TX_BUFFER_SIZE;
    case RequestState::QUEUED:
    default:
      return 0;
  }
}

uart::UARTFlushResult Arbiter::flush(uint8_t index) const {
  const Request &req = this->requests_[index];
  if (req.dropping) {
    return uart::UARTFlushResult::UART_FLUSH_RESULT_FAILED;
  }
  if (req.state == RequestState::OPEN || req.state == RequestState::QUEUED || req.sent < req.len) {
    // Still waiting for the line.
    return uart::UARTFlushResult::UART_FLUSH_RESULT_TIMEOUT;
  }
  return this->split_->parent_->flush();
}

void Arbiter::dump_config() const {
  ESP_LOGCONFIG(TAG,
                "  Mode: arbitrate\n"
                "  Send Wait Time: %" PRIu32 " ms\n"
                "  Turnaround Time: %" PRIu32 " ms\n"
                "  Request End: %" PRIu32 " us quiet\n"
                "  Reply End: %" PRIu32 " us quiet",
                this->send_wait_us_ / US_PER_MS, this->turnaround_us_ / US_PER_MS, this->request_gap_us_,
                this->reply_gap_us_);
}

}  // namespace esphome::uart_split

#endif  // USE_UART_SPLIT_ARBITER
