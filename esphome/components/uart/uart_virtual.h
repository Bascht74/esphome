#pragma once

#include "uart_component.h"
#include "esphome/core/helpers.h"

#include <cstddef>
#include <cstdint>

namespace esphome::uart {

/// Takes a block of bytes in one call, so the writer's frame boundaries survive.
class UARTSink {
 public:
  virtual void on_block(const uint8_t *data, size_t len) = 0;
};

/// A UART without a wire. Bytes for its reader arrive through inject_rx(); write_array() hands each block whole to
/// the TX sink. Two of them, each the other's TX sink, behave like a null-modem cable.
/// Reads never wait: a short read_array() or an empty peek_byte() returns false and consumes nothing.
/// Main loop only.
class VirtualUARTComponent : public UARTComponent, public UARTSink {
 public:
  /// The RX ring is allocated here, once; 0 means nothing is kept for a reader that is not attached.
  explicit VirtualUARTComponent(uint16_t rx_buffer_size);

  /// Bytes for this UART's reader: an attached reader gets the block at once, else it goes into the RX ring.
  /// Returns false when the ring has no room for all of it; nothing is kept then.
  bool inject_rx(const uint8_t *data, size_t len);
  /// Where write_array() hands each block. Without one, writes are dropped.
  void set_tx_sink(UARTSink *sink) { this->tx_sink_ = sink; }
  /// A reader that takes every received block as it arrives, so nothing waits in the ring.
  void set_rx_sink(UARTSink *sink) { this->rx_sink_ = sink; }

  /// As another UART's TX sink: what that UART writes, this one receives.
  void on_block(const uint8_t *data, size_t len) override { this->inject_rx(data, len); }

  void write_array(const uint8_t *data, size_t len) override;
  bool peek_byte(uint8_t *data) override;
  bool read_array(uint8_t *data, size_t len) override;
  size_t available() override { return this->rx_.size(); }
  size_t available_for_write() override { return this->tx_sink_ != nullptr ? SIZE_MAX : 0; }
  UARTFlushResult flush() override;
#if defined(USE_ESP8266) || defined(USE_ESP32)
  using UARTComponent::load_settings;
  // Nothing is clocked, so there is nothing to apply.
  void load_settings(bool dump_config) override {}
#endif

 protected:
  void check_logger_conflict() override {}

  UARTSink *tx_sink_{nullptr};
  UARTSink *rx_sink_{nullptr};
  FixedRingBuffer<uint8_t> rx_;
};

}  // namespace esphome::uart
