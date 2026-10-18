#include <gtest/gtest.h>

#include <vector>

#include "esphome/components/uart_split/arbiter.h"

#ifdef USE_HOST

namespace esphome::uart_split::testing {

using Bytes = std::vector<uint8_t>;

/// The UART that is split: records what goes on the line, hands out what the device sent.
class LineUart : public uart::UARTComponent {
 public:
  void write_array(const uint8_t *data, size_t len) override { this->tx_.insert(this->tx_.end(), data, data + len); }
  bool peek_byte(uint8_t *data) override {
    if (this->rx_.empty()) {
      return false;
    }
    *data = this->rx_.front();
    return true;
  }
  bool read_array(uint8_t *data, size_t len) override {
    if (this->rx_.size() < len) {
      return false;
    }
    std::copy(this->rx_.begin(), this->rx_.begin() + len, data);
    this->rx_.erase(this->rx_.begin(), this->rx_.begin() + len);
    return true;
  }
  size_t available() override { return this->rx_.size(); }
  size_t available_for_write() override { return this->room_; }
  void set_rx_full_threshold(size_t rx_full_threshold) override { this->rx_full_threshold_ = rx_full_threshold; }
  void set_rx_timeout(size_t rx_timeout) override { this->rx_timeout_ = rx_timeout; }
  uart::UARTFlushResult flush() override { return uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS; }

  Bytes tx_;
  Bytes rx_;
  size_t room_{SIZE_MAX};

 protected:
  void check_logger_conflict() override {}
};

// Outputs by index, in the order they are added.
static constexpr uint8_t A = 0;
static constexpr uint8_t B = 1;
static constexpr uint8_t C = 2;

static const Bytes REQUEST_A(8, 0xA1);
static const Bytes REQUEST_B(8, 0xB1);
static const Bytes REQUEST_C(8, 0xC1);
static const Bytes REPLY_A(9, 0xA2);
static const Bytes REPLY_B(9, 0xB2);
static const Bytes REPLY_C(9, 0xC2);

static Bytes join(std::initializer_list<Bytes> parts) {
  Bytes all;
  for (const Bytes &part : parts) {
    all.insert(all.end(), part.begin(), part.end());
  }
  return all;
}

/// Three writers and a sniffer that sees both directions. Times are in ms.
/// The line runs at 10 000 baud 8N1: 10 bits a character, so one byte takes 1 ms.
/// A request or a reply ends after 10 ms without a byte; send_wait_time is 1000 ms, turnaround_time 50 ms.
class Arbitrate : public ::testing::Test {
 protected:
  void SetUp() override {
    this->line_.set_baud_rate(10000);
    this->sniffer_.set_rx_only(true);
    this->sniffer_.set_mirror_tx(true);
    for (UartSplitOutput *output : {&this->a_, &this->b_, &this->c_, &this->sniffer_}) {
      this->split_.add_output(output);
    }
    this->split_.set_arbiter(&this->arbiter_);
    this->split_.setup();
    this->arbiter_.set_timing(1000, 10000, 10000);
  }

  /// One pass of the main loop at every ms up to and including `ms`.
  void run_until(uint32_t ms) {
    while (this->now_ms_ < ms) {
      this->now_ms_++;
      this->arbiter_.loop(this->now_ms_ * 1000);
    }
  }
  void send(uint8_t index, const Bytes &request, uint32_t ms) {
    this->run_until(ms);
    this->arbiter_.write(index, request.data(), request.size(), ms * 1000);
  }
  /// The device answers; the next pass reads it.
  void reply(const Bytes &reply, uint32_t ms) {
    this->run_until(ms - 1);
    this->line_.rx_.insert(this->line_.rx_.end(), reply.begin(), reply.end());
    this->run_until(ms);
  }
  static Bytes received(UartSplitOutput &output) {
    Bytes got(output.available());
    if (!got.empty()) {
      EXPECT_TRUE(output.read_array(got.data(), got.size()));
    }
    return got;
  }

  LineUart line_;
  UartSplit split_{&this->line_};
  UartSplitOutput a_{&this->split_};
  UartSplitOutput b_{&this->split_};
  UartSplitOutput c_{&this->split_};
  UartSplitOutput sniffer_{&this->split_};
  Arbiter arbiter_{&this->split_, 1000, 50};
  uint32_t now_ms_{0};
};

TEST_F(Arbitrate, ReplyGoesOnlyToTheWriterThatAsked) {
  this->send(A, REQUEST_A, 0);  // The line is free: out at once.
  this->send(B, REQUEST_B, 1);  // The line is busy: B waits.
  EXPECT_EQ(this->line_.tx_, REQUEST_A);
  this->reply(REPLY_A, 30);
  EXPECT_EQ(received(this->a_), REPLY_A);
  EXPECT_EQ(received(this->b_), Bytes{});
  // The reply ends at 40 (10 ms quiet), then 50 ms turnaround.
  this->run_until(89);
  EXPECT_EQ(this->line_.tx_, REQUEST_A);
  this->run_until(90);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, NoReplyGivesTheLineToTheNextWriter) {
  this->send(A, REQUEST_A, 0);  // On the line until 8.
  this->send(B, REQUEST_B, 1);
  // Give up at 8 + 1000, then 50 ms turnaround.
  this->run_until(1057);
  EXPECT_EQ(this->line_.tx_, REQUEST_A);
  this->run_until(1058);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, LateReplyReachesNoWriterAndDelaysTheNext) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->reply(REPLY_A, 1020);  // A was given up at 1008.
  EXPECT_EQ(received(this->a_), Bytes{});
  EXPECT_EQ(received(this->b_), Bytes{});
  EXPECT_EQ(received(this->sniffer_), join({REQUEST_A, REPLY_A}));
  // The turnaround time starts again at 1020.
  this->run_until(1069);
  EXPECT_EQ(this->line_.tx_, REQUEST_A);
  this->run_until(1070);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, WaitingRequestsGoOutInTheOrderTheyWereWritten) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->send(C, REQUEST_C, 2);
  this->reply(REPLY_A, 30);   // Ends at 40: B goes out at 90.
  this->reply(REPLY_B, 100);  // Ends at 110: C goes out at 160.
  this->run_until(160);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B, REQUEST_C}));
  EXPECT_EQ(received(this->b_), REPLY_B);
  EXPECT_EQ(received(this->c_), Bytes{});
}

TEST_F(Arbitrate, NextRequestOfTheOwnerWaitsForTheOthers) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->reply(REPLY_A, 30);
  this->send(A, REQUEST_A, 35);  // While its reply still counts as running.
  this->reply(REPLY_B, 100);
  this->run_until(160);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B, REQUEST_A}));
}

TEST_F(Arbitrate, OwnerAskingAgainWithoutAReplyWaitsForTheOthers) {
  this->send(A, REQUEST_A, 0);  // To a device that never answers.
  this->send(B, REQUEST_B, 1);
  this->send(A, REQUEST_A, 300);  // A gave up on its own and asks again.
  // Given up at 8 + 1000, then 50 ms turnaround: B first, A's second request waits behind it.
  this->run_until(1058);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, ReplyWithAShortPauseStaysOneReply) {
  const Bytes first(4, 0xA2);
  const Bytes rest(5, 0xA2);
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->reply(first, 30);
  this->reply(rest, 38);  // 8 ms quiet: less than the 10 ms that end a reply.
  EXPECT_EQ(received(this->a_), join({first, rest}));
  EXPECT_EQ(received(this->b_), Bytes{});
  // The reply ends at 48, then 50 ms turnaround.
  this->run_until(97);
  EXPECT_EQ(this->line_.tx_, REQUEST_A);
  this->run_until(98);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, RoomTellsAWriterWhenToWait) {
  this->line_.room_ = 100;
  EXPECT_EQ(this->arbiter_.available_for_write(A), 100u);  // Free line: what the UART takes.
  this->send(A, REQUEST_A, 0);
  EXPECT_EQ(this->arbiter_.available_for_write(A), 100u);  // More of the same request may follow.
  EXPECT_EQ(this->arbiter_.available_for_write(B), TX_BUFFER_SIZE);
  this->send(B, REQUEST_B, 1);
  EXPECT_EQ(this->arbiter_.available_for_write(B), TX_BUFFER_SIZE - 8);
  this->run_until(11);  // B's request is complete and waits.
  EXPECT_EQ(this->arbiter_.available_for_write(B), 0u);
  EXPECT_EQ(this->arbiter_.flush(B), uart::UARTFlushResult::UART_FLUSH_RESULT_TIMEOUT);
  EXPECT_EQ(this->arbiter_.flush(A), uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS);
}

TEST_F(Arbitrate, SecondRequestWhileTheFirstWaitsIsDropped) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->send(B, Bytes(8, 0xB9), 20);  // A writer that ignores the room it was given.
  EXPECT_EQ(this->arbiter_.flush(B), uart::UARTFlushResult::UART_FLUSH_RESULT_FAILED);
  this->reply(REPLY_A, 30);
  this->run_until(200);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, TooLongRequestIsDroppedAndTheOthersGoOn) {
  this->send(A, REQUEST_A, 0);
  this->send(B, Bytes(TX_BUFFER_SIZE + 1, 0xB1), 1);
  this->send(C, REQUEST_C, 2);
  this->reply(REPLY_A, 30);
  this->run_until(200);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_C}));
}

TEST_F(Arbitrate, RequestWrittenInTwoPiecesStaysOneRequest) {
  const Bytes first(5, 0xB1);
  const Bytes rest(3, 0xB1);
  this->send(A, first, 0);
  this->send(A, rest, 5);  // On the line, before any reply.
  this->send(B, first, 6);
  this->send(B, rest, 10);  // Waiting, and less than 10 ms after the first piece.
  this->reply(REPLY_A, 30);
  this->reply(REPLY_B, 100);
  EXPECT_EQ(this->line_.tx_, join({first, rest, first, rest}));
  EXPECT_EQ(received(this->b_), REPLY_B);
}

TEST_F(Arbitrate, WaitingRequestGoesOutAsTheUartTakesIt) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->reply(REPLY_A, 30);
  this->line_.room_ = 5;
  this->run_until(90);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, Bytes(5, 0xB1)}));
  this->run_until(91);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_B}));
}

TEST_F(Arbitrate, RequestTheUartDoesNotTakeIsGivenUp) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->send(C, REQUEST_C, 2);
  this->reply(REPLY_A, 30);
  this->line_.room_ = 0;  // The UART takes nothing, e.g. a tcp_uart whose link is down.
  // B gets the line at 90 but nothing goes out: given up at 90 + 1000, then 50 ms turnaround.
  this->run_until(1100);
  this->line_.room_ = SIZE_MAX;
  this->run_until(1139);
  EXPECT_EQ(this->line_.tx_, REQUEST_A);
  this->run_until(1140);
  EXPECT_EQ(this->line_.tx_, join({REQUEST_A, REQUEST_C}));
}

TEST_F(Arbitrate, SnifferSeesTheLineInOrder) {
  this->send(A, REQUEST_A, 0);
  this->send(B, REQUEST_B, 1);
  this->reply(REPLY_A, 30);
  this->reply(REPLY_B, 100);
  EXPECT_EQ(received(this->sniffer_), join({REQUEST_A, REPLY_A, REQUEST_B, REPLY_B}));
}

TEST_F(Arbitrate, UnaskedBytesGoOnlyToTheSniffer) {
  this->reply(Bytes{0x55}, 5);
  EXPECT_EQ(received(this->a_), Bytes{});
  EXPECT_EQ(received(this->b_), Bytes{});
  EXPECT_EQ(received(this->sniffer_), Bytes{0x55});
}

TEST_F(Arbitrate, FullWriterLosesItsReplyButHoldsNothingBack) {
  const Bytes full(RX_BUFFER_SIZE, 0x00);
  ASSERT_TRUE(this->a_.inject_rx(full.data(), full.size()));
  this->send(A, REQUEST_A, 0);
  this->reply(REPLY_A, 30);
  EXPECT_EQ(this->line_.available(), 0u);
  EXPECT_EQ(this->a_.available(), RX_BUFFER_SIZE);
  EXPECT_EQ(received(this->sniffer_), join({REQUEST_A, REPLY_A}));
}

TEST(ArbiterTiming, FrameDelayIsThreeAndAHalfCharacters) {
  LineUart line;
  line.set_baud_rate(9600);  // 8N1: 10 bits, 1.04 ms a character.
  EXPECT_EQ(char_time_us(&line), 1042u);
  EXPECT_EQ(frame_delay_us(&line), 3646u);
  line.set_baud_rate(115200);  // 3.5 characters are 0.3 ms: the 1.75 ms floor applies.
  EXPECT_EQ(frame_delay_us(&line), 1750u);
}

TEST(ArbiterTiming, ReplyGapCoversWhatTheDriverHoldsBack) {
  LineUart line;
  line.set_baud_rate(10000);  // 1 ms a character.
  // No receive threshold: bytes may come in chunks, 50 ms is allowed for that.
  EXPECT_EQ(reply_gap_us(&line), 3501u + 50000u);
  // Up to 20 characters wait for the threshold, 2 more for the idle timeout.
  line.set_rx_full_threshold(20);
  line.set_rx_timeout(2);
  EXPECT_EQ(reply_gap_us(&line), 3501u + 22000u + 1u);
}

}  // namespace esphome::uart_split::testing

#endif
