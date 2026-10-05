#include "esphome/components/uart/uart_virtual.h"

#ifdef USE_HOST

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome::uart::testing {

// Records each block it is handed, so a test can see the boundaries.
class BlockRecorder : public UARTSink {
 public:
  void on_block(const uint8_t *data, size_t len) override { this->blocks.emplace_back(data, data + len); }

  std::vector<std::vector<uint8_t>> blocks;
};

static constexpr uint8_t FRAME[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC5, 0xCD};

TEST(VirtualUART, InjectedBytesAreReadInOrder) {
  VirtualUARTComponent uart(16);
  EXPECT_EQ(uart.get_rx_buffer_size(), 16u);
  EXPECT_TRUE(uart.inject_rx(FRAME, sizeof(FRAME)));
  EXPECT_EQ(uart.available(), sizeof(FRAME));
  uint8_t peeked = 0;
  EXPECT_TRUE(uart.peek_byte(&peeked));
  EXPECT_EQ(peeked, 0x01);
  uint8_t out[sizeof(FRAME)]{};
  EXPECT_TRUE(uart.read_array(out, 3));
  EXPECT_TRUE(uart.read_array(out + 3, sizeof(FRAME) - 3));
  EXPECT_EQ(std::vector<uint8_t>(out, out + sizeof(out)), std::vector<uint8_t>(FRAME, FRAME + sizeof(FRAME)));
  EXPECT_EQ(uart.available(), 0u);
}

TEST(VirtualUART, ShortReadAndEmptyPeekFailWithoutConsuming) {
  VirtualUARTComponent uart(16);
  uint8_t byte = 0;
  EXPECT_FALSE(uart.peek_byte(&byte));
  uart.inject_rx(FRAME, 2);
  uint8_t out[4]{};
  EXPECT_FALSE(uart.read_array(out, 3));
  EXPECT_EQ(uart.available(), 2u);
}

TEST(VirtualUART, BlockThatDoesNotFitIsRefusedWhole) {
  VirtualUARTComponent uart(10);
  EXPECT_TRUE(uart.inject_rx(FRAME, sizeof(FRAME)));
  EXPECT_FALSE(uart.inject_rx(FRAME, 3));
  EXPECT_EQ(uart.available(), sizeof(FRAME));
  VirtualUARTComponent no_ring(0);
  EXPECT_FALSE(no_ring.inject_rx(FRAME, 1));
  EXPECT_EQ(no_ring.available(), 0u);
}

TEST(VirtualUART, RingWrapsAround) {
  VirtualUARTComponent uart(10);
  uint8_t out[sizeof(FRAME)]{};
  for (int round = 0; round < 5; round++) {
    ASSERT_TRUE(uart.inject_rx(FRAME, sizeof(FRAME)));
    ASSERT_TRUE(uart.read_array(out, sizeof(FRAME)));
    EXPECT_EQ(out[sizeof(FRAME) - 1], 0xCD);
  }
}

TEST(VirtualUART, WriteHandsTheWholeBlockToTheSink) {
  VirtualUARTComponent uart(0);
  BlockRecorder sink;
  uart.set_tx_sink(&sink);
  uart.write_array(FRAME, sizeof(FRAME));
  ASSERT_EQ(sink.blocks.size(), 1u);
  EXPECT_EQ(sink.blocks[0].size(), sizeof(FRAME));
  EXPECT_EQ(uart.available_for_write(), SIZE_MAX);
  EXPECT_EQ(uart.flush(), UARTFlushResult::UART_FLUSH_RESULT_SUCCESS);
}

TEST(VirtualUART, WithoutSinkWritesAreDroppedAndFlushFails) {
  VirtualUARTComponent uart(0);
  uart.write_array(FRAME, sizeof(FRAME));
  EXPECT_EQ(uart.available_for_write(), 0u);
  EXPECT_EQ(uart.flush(), UARTFlushResult::UART_FLUSH_RESULT_FAILED);
}

TEST(VirtualUART, PairedUartsActLikeANullModem) {
  VirtualUARTComponent a(16);
  VirtualUARTComponent b(16);
  a.set_tx_sink(&b);
  b.set_tx_sink(&a);
  a.write_array(FRAME, sizeof(FRAME));
  EXPECT_EQ(b.available(), sizeof(FRAME));
  EXPECT_EQ(a.available(), 0u);
  b.write_byte(0x42);
  uint8_t byte = 0;
  EXPECT_TRUE(a.read_byte(&byte));
  EXPECT_EQ(byte, 0x42);
}

TEST(VirtualUART, AttachedReaderGetsBlocksAndTheRingStaysEmpty) {
  VirtualUARTComponent uart(16);
  BlockRecorder reader;
  uart.set_rx_sink(&reader);
  EXPECT_TRUE(uart.inject_rx(FRAME, sizeof(FRAME)));
  EXPECT_TRUE(uart.inject_rx(FRAME, 3));
  ASSERT_EQ(reader.blocks.size(), 2u);
  EXPECT_EQ(reader.blocks[0].size(), sizeof(FRAME));
  EXPECT_EQ(reader.blocks[1].size(), 3u);
  EXPECT_EQ(uart.available(), 0u);
}

}  // namespace esphome::uart::testing

#endif  // USE_HOST
