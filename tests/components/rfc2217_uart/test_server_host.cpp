#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "fakes.h"

namespace esphome::rfc2217_uart::testing {
namespace {

class Server : public Rfc2217Server {
 public:
  Server(Pipe *pipe, FakeSerial *serial) {
    this->set_tcp_uart(pipe);
    this->set_uart_parent(serial);
  }
  size_t pending() const { return this->to_serial_len_; }

  int loads_{0};
  bool can_load_{true};

 protected:
  bool load_serial() override {
    this->loads_++;
    return this->can_load_;
  }
};

const uint8_t WILL_COM_PORT[] = {0xFF, 0xFB, 0x2C};

// Connected and COM-PORT accepted.
void accept(Pipe &pipe, Server &server) {
  server.loop();
  pipe.feed(WILL_COM_PORT);
  server.loop();
  pipe.clear();
}

// One COM-PORT command from the client.
void command(Pipe &pipe, uint8_t code, uint8_t value) {
  const uint8_t sub[] = {0xFF, 0xFA, 0x2C, code, value, 0xFF, 0xF0};
  pipe.feed(sub);
}

template<size_t N> void expect_only(const Pipe &pipe, const uint8_t (&want)[N]) {
  ASSERT_EQ(pipe.n_, N);
  EXPECT_EQ(std::memcmp(pipe.buf_, want, N), 0);
}

TEST(Rfc2217Server, ConnectRequestsComPort) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  server.dump_config();
  // Stale bytes from before the connect are dropped.
  const uint8_t stale[] = {0x01, 0x02};
  serial.feed(stale, sizeof(stale));
  server.loop();
  const uint8_t want[] = {0xFF, 0xFB, 0x00, 0xFF, 0xFD, 0x00, 0xFF, 0xFD, 0x2C};
  expect_only(pipe, want);
  EXPECT_EQ(serial.rx_n_, 0u);
}

TEST(Rfc2217Server, BaudChangeIsLoadedIntoTheUart) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  const uint8_t set[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x00, 0x4B, 0x00, 0xFF, 0xF0};
  pipe.feed(set);
  server.loop();
  EXPECT_EQ(serial.get_baud_rate(), 19200u);
  EXPECT_EQ(server.loads_, 1);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 101, 0x00, 0x00, 0x4B, 0x00, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, QueryIsAnsweredWithTheValueInUse) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  const uint8_t query[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF0};
  pipe.feed(query);
  // The value unchanged: nothing to load.
  command(pipe, 0x02, 8);
  server.loop();
  EXPECT_EQ(server.loads_, 0);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 101,  0x00, 0x00, 0x25, 0x80, 0xFF,
                          0xF0, 0xFF, 0xFA, 0x2C, 102,  0x08, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, BaudRateOutOfRangeIsNotLoaded) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  // 50 baud, then 6 Mbaud.
  const uint8_t slow[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x32, 0xFF, 0xF0};
  const uint8_t fast[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x5B, 0x8D, 0x80, 0xFF, 0xF0};
  pipe.feed(slow);
  pipe.feed(fast);
  server.loop();
  EXPECT_EQ(server.loads_, 0);
  EXPECT_EQ(serial.get_baud_rate(), 9600u);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 101, 0x00, 0x00, 0x25, 0x80, 0xFF, 0xF0,
                          0xFF, 0xFA, 0x2C, 101, 0x00, 0x00, 0x25, 0x80, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, FixedUartAnswersTheValueInUse) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  server.can_load_ = false;
  accept(pipe, server);
  const uint8_t set[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x00, 0x4B, 0x00, 0xFF, 0xF0};
  pipe.feed(set);
  command(pipe, 0x02, 7);
  server.loop();
  EXPECT_EQ(serial.get_baud_rate(), 9600u);
  EXPECT_EQ(serial.get_data_bits(), 8u);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 101,  0x00, 0x00, 0x25, 0x80, 0xFF,
                          0xF0, 0xFF, 0xFA, 0x2C, 102,  0x08, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, UnsupportedValuesAreAnsweredWithTheValueInUse) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  // MARK parity, 1.5 stop bits, 9 data bits.
  command(pipe, 0x03, 4);
  command(pipe, 0x04, 3);
  command(pipe, 0x02, 9);
  server.loop();
  EXPECT_EQ(server.loads_, 0);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 103,  0x01, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 104,
                          0x01, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 102,  0x08, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, LineSettingsAreLoaded) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  command(pipe, 0x03, 3);
  command(pipe, 0x03, 2);
  command(pipe, 0x03, 1);
  command(pipe, 0x03, 2);
  command(pipe, 0x04, 2);
  command(pipe, 0x02, 7);
  server.loop();
  EXPECT_EQ(serial.get_parity(), uart::UART_CONFIG_PARITY_ODD);
  EXPECT_EQ(serial.get_stop_bits(), 2u);
  EXPECT_EQ(serial.get_data_bits(), 7u);
  EXPECT_EQ(server.loads_, 6);
  const uint8_t even[] = {0xFF, 0xFA, 0x2C, 103, 0x03, 0xFF, 0xF0};
  const uint8_t odd[] = {0xFF, 0xFA, 0x2C, 103, 0x02, 0xFF, 0xF0};
  const uint8_t stop[] = {0xFF, 0xFA, 0x2C, 104, 0x02, 0xFF, 0xF0};
  EXPECT_TRUE(pipe.sent(even));
  EXPECT_TRUE(pipe.sent(odd));
  EXPECT_TRUE(pipe.sent(stop));
}

TEST(Rfc2217Server, ControlCommandsGetTheTrueValue) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  // XON/XOFF, BREAK on, inbound XON/XOFF, DCD flow, DTR flow; DTR on and RTS on get no answer.
  command(pipe, 0x05, 2);
  command(pipe, 0x05, 5);
  command(pipe, 0x05, 15);
  command(pipe, 0x05, 17);
  command(pipe, 0x05, 18);
  command(pipe, 0x05, 8);
  command(pipe, 0x05, 11);
  command(pipe, 0x05, 20);
  server.loop();
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 105,  0x01, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 105,  0x06,
                          0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 105,  0x0E, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C,
                          105,  0x01, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 105,  0x0E, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, StateMasksAreAnsweredZero) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  command(pipe, 10, 0xFE);
  command(pipe, 11, 0xFE);
  server.loop();
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 110, 0x00, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 111, 0x00, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, PurgeTransmitDropsThePayloadBeforeIt) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  serial.room_ = 0;
  const uint8_t data[] = {0x41, 0x42};
  pipe.feed(data);
  server.loop();
  EXPECT_EQ(server.pending(), 2u);
  pipe.feed(data);
  command(pipe, 12, 2);
  server.loop();
  EXPECT_EQ(server.pending(), 0u);
  EXPECT_EQ(serial.n_, 0u);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 112, 0x02, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, PurgeReceiveDropsTheUartInput) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  const uint8_t data[] = {0x01, 0x02, 0x03};
  serial.feed(data, sizeof(data));
  command(pipe, 12, 3);
  // Out of range: no answer.
  command(pipe, 12, 0);
  command(pipe, 12, 4);
  server.loop();
  EXPECT_EQ(serial.rx_n_, 0u);
  const uint8_t want[] = {0xFF, 0xFA, 0x2C, 112, 0x03, 0xFF, 0xF0};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, XonXoffAndDoubledIacReachTheUart) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  const uint8_t stream[] = {0x11, 0x13, 0xFF, 0xFF};
  pipe.feed(stream);
  server.loop();
  const uint8_t want[] = {0x11, 0x13, 0xFF};
  ASSERT_EQ(serial.n_, sizeof(want));
  EXPECT_EQ(std::memcmp(serial.buf_, want, sizeof(want)), 0);
  EXPECT_EQ(pipe.n_, 0u);
}

TEST(Rfc2217Server, UartIacIsDoubled) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  const uint8_t data[] = {0x41, 0xFF, 0x13};
  serial.feed(data, sizeof(data));
  server.loop();
  const uint8_t want[] = {0x41, 0xFF, 0xFF, 0x13};
  expect_only(pipe, want);
}

TEST(Rfc2217Server, WritesArePacedToTheUartRoom) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  serial.room_ = 2;
  const uint8_t data[] = {1, 2, 3, 4, 5};
  pipe.feed(data);
  server.loop();
  EXPECT_EQ(serial.n_, 2u);
  server.loop();
  EXPECT_EQ(serial.n_, 4u);
  server.loop();
  EXPECT_EQ(serial.n_, 5u);
  EXPECT_EQ(std::memcmp(serial.buf_, data, sizeof(data)), 0);
}

TEST(Rfc2217Server, UnknownRoomIsPacedByTheBaudRate) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  serial.room_ = SIZE_MAX;
  const uint8_t data[] = {1, 2, 3};
  pipe.feed(data);
  // No time has passed in the test, so each pass writes the minimum of one byte.
  server.loop();
  EXPECT_EQ(serial.n_, 1u);
}

TEST(Rfc2217Server, FullBufferSuspendsTheClient) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  serial.room_ = 0;
  uint8_t data[200];
  std::memset(data, 0x30, sizeof(data));
  pipe.feed(data, sizeof(data));
  server.loop();
  server.loop();
  EXPECT_EQ(server.pending(), sizeof(data));
  const uint8_t suspend[] = {0xFF, 0xFA, 0x2C, 108, 0xFF, 0xF0};
  expect_only(pipe, suspend);
  pipe.clear();
  serial.room_ = 1024;
  server.loop();
  const uint8_t resume[] = {0xFF, 0xFA, 0x2C, 109, 0xFF, 0xF0};
  expect_only(pipe, resume);
}

TEST(Rfc2217Server, NoFlowControlWithoutComPort) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  server.loop();
  pipe.clear();
  serial.room_ = 0;
  uint8_t data[200];
  std::memset(data, 0x30, sizeof(data));
  pipe.feed(data, sizeof(data));
  server.loop();
  server.loop();
  EXPECT_EQ(pipe.n_, 0u);
}

TEST(Rfc2217Server, ClientSuspendHoldsTheUartBytes) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  const uint8_t suspend[] = {0xFF, 0xFA, 0x2C, 0x08, 0xFF, 0xF0};
  pipe.feed(suspend);
  server.loop();
  const uint8_t data[] = {0x41};
  serial.feed(data, sizeof(data));
  server.loop();
  EXPECT_EQ(pipe.n_, 0u);
  const uint8_t resume[] = {0xFF, 0xFA, 0x2C, 0x09, 0xFF, 0xF0};
  pipe.feed(resume);
  server.loop();
  expect_only(pipe, data);
}

TEST(Rfc2217Server, ShortAndForeignSubnegotiationsAreIgnored) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  // SET-BAUDRATE with two bytes, SET-DATASIZE without a value, an unknown command, TERMINAL-TYPE.
  const uint8_t stream[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x00, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 0x02, 0xFF, 0xF0,
                            0xFF, 0xFA, 0x2C, 0x00, 0x41, 0xFF, 0xF0, 0xFF, 0xFA, 0x18, 0x01, 0xFF, 0xF0};
  pipe.feed(stream);
  server.loop();
  EXPECT_EQ(pipe.n_, 0u);
  EXPECT_EQ(serial.n_, 0u);
}

TEST(Rfc2217Server, LinkEdgesClearThePendingPayload) {
  Pipe pipe;
  FakeSerial serial;
  Server server(&pipe, &serial);
  accept(pipe, server);
  serial.room_ = 0;
  const uint8_t data[] = {0x41};
  pipe.feed(data);
  server.loop();
  EXPECT_EQ(server.pending(), 1u);
  pipe.up_ = false;
  server.loop();
  EXPECT_EQ(server.pending(), 0u);
  pipe.up_ = true;
  server.loop();
  serial.room_ = 1024;
  server.loop();
  EXPECT_EQ(serial.n_, 0u);
}

}  // namespace
}  // namespace esphome::rfc2217_uart::testing
