#pragma once

#include "esphome/core/defines.h"
#ifdef USE_UART_SPLIT_ARBITER

#include "uart_split.h"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace esphome::uart_split {

/// One request per writer: the longest Modbus RTU frame.
static constexpr size_t TX_BUFFER_SIZE = 256;

/// Time of one character on the line, in microseconds.
uint32_t char_time_us(uart::UARTComponent *uart);
/// Quiet time that ends an RTU frame: 3.5 characters, at least 1.75 ms.
uint32_t frame_delay_us(uart::UARTComponent *uart);
/// Quiet time that ends a reply: the frame delay plus the time the driver may hold received bytes back.
uint32_t reply_gap_us(uart::UARTComponent *uart);

/// Lets one writer at a time use the line. Its request goes out, the reply goes back to it alone, and the
/// requests of the other writers wait in turn. Every time is a parameter, so the caller owns the clock.
class Arbiter {
 public:
  Arbiter(UartSplit *split, uint32_t send_wait_ms, uint32_t turnaround_ms)
      : split_(split), send_wait_us_(send_wait_ms * 1000), turnaround_us_(turnaround_ms * 1000) {}

  /// Allocates one request buffer per writer and takes the timing from the UART that is split.
  void setup();
  /// request_gap_us: quiet time on a writer that ends its request; reply_gap_us: on the line, ends a reply.
  void set_timing(uint32_t char_us, uint32_t request_gap_us, uint32_t reply_gap_us);

  void loop(uint32_t now_us);
  void write(uint8_t index, const uint8_t *data, size_t len, uint32_t now_us);
  size_t available_for_write(uint8_t index) const;
  uart::UARTFlushResult flush(uint8_t index) const;
  void dump_config() const;

 protected:
  enum class Line : uint8_t {
    IDLE,       ///< Free for the next request.
    SENT,       ///< The owner's request is on the line; no reply yet.
    RECEIVING,  ///< The reply is coming in.
    GAP,        ///< Turnaround time before the next request.
  };
  enum class RequestState : uint8_t {
    FREE,
    OPEN,    ///< The writer is still writing it.
    QUEUED,  ///< Complete, waits for the line.
    ACTIVE,  ///< On the line; its reply goes to this writer.
  };
  struct Request {
    uint8_t *data{nullptr};
    uint32_t last_write_us{0};
    uint16_t len{0};
    uint16_t sent{0};
    RequestState state{RequestState::FREE};
    /// Writes are dropped until the writer has been quiet for the request gap.
    bool dropping{false};
  };
  static constexpr uint8_t NO_OWNER = 0xFF;

  void receive_(uint32_t now_us);
  void close_requests_(uint32_t now_us);
  void start_(uint8_t index, uint32_t now_us);
  void send_(const uint8_t *data, size_t len, uint32_t now_us);
  void hand_queued_bytes_(uint32_t now_us);
  void enter_gap_(uint32_t from_us);
  void drop_(uint8_t index);

  UartSplit *split_;
  std::unique_ptr<Request[]> requests_;
  std::unique_ptr<uint8_t[]> buffers_;
  uint32_t send_wait_us_;
  uint32_t turnaround_us_;
  uint32_t char_us_{0};
  uint32_t request_gap_us_{0};
  uint32_t reply_gap_us_{0};
  /// When the last byte handed to the UART has left the line.
  uint32_t tx_end_us_{0};
  uint32_t last_rx_us_{0};
  uint32_t gap_from_us_{0};
  /// Indexes of complete requests in the order they were completed; each writer is in it at most once.
  uint8_t queue_[MAX_OUTPUTS]{};
  uint8_t queue_head_{0};
  uint8_t queue_count_{0};
  uint8_t owner_{NO_OWNER};
  Line line_{Line::IDLE};
};

}  // namespace esphome::uart_split

#endif  // USE_UART_SPLIT_ARBITER
