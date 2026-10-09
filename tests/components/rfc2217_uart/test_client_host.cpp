#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "fakes.h"

namespace esphome::rfc2217_uart::testing {
namespace {

class Client : public Rfc2217Client {
 public:
  explicit Client(Pipe *pipe) {
    this->set_tcp_uart(pipe);
    this->set_baud_rate(9600);
    this->set_data_bits(8);
    this->set_parity(uart::UART_CONFIG_PARITY_NONE);
    this->set_stop_bits(1);
  }
  // load_settings() on ESP8266 and ESP32.
  void send_settings() { this->send_settings_(); }
  bool com_port() const { return this->com_port_(); }
};

const uint8_t DO_COM_PORT[] = {0xFF, 0xFD, 0x2C};
const uint8_t SUSPEND[] = {0xFF, 0xFA, 0x2C, 0x08, 0xFF, 0xF0};
const uint8_t RESUME[] = {0xFF, 0xFA, 0x2C, 0x09, 0xFF, 0xF0};
const uint8_t SERVER_SUSPEND[] = {0xFF, 0xFA, 0x2C, 108, 0xFF, 0xF0};
const uint8_t SERVER_RESUME[] = {0xFF, 0xFA, 0x2C, 109, 0xFF, 0xF0};

// Connected, COM-PORT accepted and the first settings sent.
void accept(Pipe &pipe, Client &client) {
  client.loop();
  pipe.feed(DO_COM_PORT);
  client.loop();
  pipe.clear();
}

TEST(Rfc2217Client, ConnectOffersBinaryAndComPort) {
  Pipe pipe;
  Client client(&pipe);
  client.dump_config();
  client.loop();
  const uint8_t want[] = {0xFF, 0xFB, 0x00, 0xFF, 0xFD, 0x00, 0xFF, 0xFB, 0x2C};
  ASSERT_EQ(pipe.n_, sizeof(want));
  EXPECT_EQ(std::memcmp(pipe.buf_, want, sizeof(want)), 0);
  EXPECT_FALSE(client.com_port());
}

TEST(Rfc2217Client, SettingsGoOutWhenTheServerAcceptsComPort) {
  Pipe pipe;
  Client client(&pipe);
  client.set_parity(uart::UART_CONFIG_PARITY_EVEN);
  client.loop();
  pipe.clear();
  client.loop();
  EXPECT_EQ(pipe.n_, 0u);
  pipe.feed(DO_COM_PORT);
  client.loop();
  EXPECT_TRUE(client.com_port());
  const uint8_t baud[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x00, 0x25, 0x80, 0xFF, 0xF0};
  const uint8_t data_bits[] = {0xFF, 0xFA, 0x2C, 0x02, 0x08, 0xFF, 0xF0};
  const uint8_t parity[] = {0xFF, 0xFA, 0x2C, 0x03, 0x03, 0xFF, 0xF0};
  const uint8_t stop_bits[] = {0xFF, 0xFA, 0x2C, 0x04, 0x01, 0xFF, 0xF0};
  EXPECT_TRUE(pipe.sent(baud));
  EXPECT_TRUE(pipe.sent(data_bits));
  EXPECT_TRUE(pipe.sent(parity));
  EXPECT_TRUE(pipe.sent(stop_bits));
  // The accepted offer is not answered.
  EXPECT_EQ(pipe.n_, sizeof(baud) + sizeof(data_bits) + sizeof(parity) + sizeof(stop_bits));
}

TEST(Rfc2217Client, LoadSettingsSendsTheNewBaudRate) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  client.set_baud_rate(115200);
  client.send_settings();
  const uint8_t baud[] = {0xFF, 0xFA, 0x2C, 0x01, 0x00, 0x01, 0xC2, 0x00, 0xFF, 0xF0};
  EXPECT_TRUE(pipe.sent(baud));
}

TEST(Rfc2217Client, SettingsWaitForRoomAndComPort) {
  Pipe pipe;
  Client client(&pipe);
  client.loop();
  pipe.clear();
  client.send_settings();
  EXPECT_EQ(pipe.n_, 0u);
  pipe.feed(DO_COM_PORT);
  pipe.room_ = 10;
  client.loop();
  EXPECT_EQ(pipe.n_, 0u);
  pipe.room_ = 1024;
  client.loop();
  EXPECT_GT(pipe.n_, 0u);
}

TEST(Rfc2217Client, RefusedComPortSendsNoSettings) {
  Pipe pipe;
  Client client(&pipe);
  client.loop();
  pipe.clear();
  const uint8_t dont[] = {0xFF, 0xFE, 0x2C};
  pipe.feed(dont);
  client.loop();
  EXPECT_FALSE(client.com_port());
  client.send_settings();
  EXPECT_EQ(pipe.n_, 0u);
}

TEST(Rfc2217Client, PayloadIacIsDoubled) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  const uint8_t data[] = {0x01, 0xFF, 0x13, 0x11};
  client.write_array(data, sizeof(data));
  const uint8_t want[] = {0x01, 0xFF, 0xFF, 0x13, 0x11};
  ASSERT_EQ(pipe.n_, sizeof(want));
  EXPECT_EQ(std::memcmp(pipe.buf_, want, sizeof(want)), 0);
  EXPECT_EQ(client.flush(), uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS);
}

TEST(Rfc2217Client, XonXoffAndDoubledIacArePayload) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  const uint8_t stream[] = {0x11, 0x13, 0xFF, 0xFF, 0x41};
  pipe.feed(stream);
  client.loop();
  uint8_t got[4];
  ASSERT_EQ(client.available(), sizeof(got));
  ASSERT_TRUE(client.read_array(got, sizeof(got)));
  const uint8_t want[] = {0x11, 0x13, 0xFF, 0x41};
  EXPECT_EQ(std::memcmp(got, want, sizeof(want)), 0);
  // Payload bytes are no flow control, so nothing went back.
  EXPECT_EQ(pipe.n_, 0u);
}

TEST(Rfc2217Client, ServerSuspendHoldsTheWritesUntilResume) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  pipe.feed(SERVER_SUSPEND);
  client.loop();
  const uint8_t data[] = {0x11, 0x13};
  client.write_array(data, sizeof(data));
  EXPECT_EQ(pipe.n_, 0u);
  EXPECT_EQ(client.flush(), uart::UARTFlushResult::UART_FLUSH_RESULT_TIMEOUT);
  EXPECT_EQ(client.available_for_write(), 256u - sizeof(data));
  pipe.feed(SERVER_RESUME);
  client.loop();
  ASSERT_EQ(pipe.n_, sizeof(data));
  EXPECT_EQ(std::memcmp(pipe.buf_, data, sizeof(data)), 0);
}

TEST(Rfc2217Client, FullTxBufferDropsTheRest) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  pipe.feed(SERVER_SUSPEND);
  client.loop();
  uint8_t data[300];
  std::memset(data, 0x30, sizeof(data));
  client.write_array(data, sizeof(data));
  EXPECT_EQ(client.available_for_write(), 0u);
  pipe.feed(SERVER_RESUME);
  client.loop();
  EXPECT_EQ(pipe.n_, 256u);
}

TEST(Rfc2217Client, FullRxSuspendsTheServer) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  uint8_t data[200];
  std::memset(data, 0x30, sizeof(data));
  pipe.feed(data, sizeof(data));
  client.loop();
  EXPECT_FALSE(pipe.sent(SUSPEND));
  client.loop();
  EXPECT_EQ(client.available(), sizeof(data));
  EXPECT_TRUE(pipe.sent(SUSPEND));
  pipe.clear();
  uint8_t got[136];
  ASSERT_TRUE(client.read_array(got, sizeof(got)));
  client.loop();
  EXPECT_TRUE(pipe.sent(RESUME));
}

TEST(Rfc2217Client, OtherOptionsAreRefused) {
  Pipe pipe;
  Client client(&pipe);
  client.loop();
  pipe.clear();
  // WILL ECHO, DO SGA, WONT 5.
  const uint8_t stream[] = {0xFF, 0xFB, 0x01, 0xFF, 0xFD, 0x03, 0xFF, 0xFC, 0x05};
  pipe.feed(stream);
  client.loop();
  const uint8_t want[] = {0xFF, 0xFE, 0x01, 0xFF, 0xFC, 0x03};
  ASSERT_EQ(pipe.n_, sizeof(want));
  EXPECT_EQ(std::memcmp(pipe.buf_, want, sizeof(want)), 0);
}

TEST(Rfc2217Client, OptionStateIsAnsweredOnce) {
  Pipe pipe;
  Client client(&pipe);
  client.loop();
  pipe.clear();
  // Both BINARY offers accepted, then accepted again.
  const uint8_t accepted[] = {0xFF, 0xFD, 0x00, 0xFF, 0xFB, 0x00, 0xFF, 0xFD, 0x00};
  pipe.feed(accepted);
  client.loop();
  EXPECT_EQ(pipe.n_, 0u);
  // WONT BINARY while on, WILL COM-PORT while off.
  const uint8_t changes[] = {0xFF, 0xFC, 0x00, 0xFF, 0xFB, 0x2C, 0xFF, 0xFC, 0x00};
  pipe.feed(changes);
  client.loop();
  const uint8_t want[] = {0xFF, 0xFE, 0x00, 0xFF, 0xFD, 0x2C};
  ASSERT_EQ(pipe.n_, sizeof(want));
  EXPECT_EQ(std::memcmp(pipe.buf_, want, sizeof(want)), 0);
}

TEST(Rfc2217Client, CommandWaitsForRoom) {
  Pipe pipe;
  Client client(&pipe);
  client.loop();
  pipe.clear();
  pipe.room_ = 5;
  const uint8_t stream[] = {0x41, 0xFF, 0xFB, 0x01};
  pipe.feed(stream);
  client.loop();
  EXPECT_EQ(client.available(), 1u);
  EXPECT_EQ(pipe.rx_n_, 3u);
  pipe.room_ = 1024;
  client.loop();
  const uint8_t want[] = {0xFF, 0xFE, 0x01};
  EXPECT_TRUE(pipe.sent(want));
}

TEST(Rfc2217Client, ServerAnswersAreNoPayload) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  // Answers that differ from the request, short ones, a line state notice and another subnegotiation.
  const uint8_t stream[] = {0xFF, 0xFA, 0x2C, 101,  0x00, 0x00, 0x12, 0xC0, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 102,
                            7,    0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 103,  2,    0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 104,
                            2,    0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 101,  0x00, 0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 102,
                            0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 103,  0xFF, 0xF0, 0xFF, 0xFA, 0x2C, 104,  0xFF, 0xF0,
                            0xFF, 0xFA, 0x2C, 106,  0x60, 0xFF, 0xF0, 0xFF, 0xFA, 0x18, 0x01, 0xFF, 0xF0};
  pipe.feed(stream);
  client.loop();
  EXPECT_EQ(client.available(), 0u);
  EXPECT_EQ(pipe.n_, 0u);
}

TEST(Rfc2217Client, LinkDownDropsTheUnsentAndUpClearsTheRest) {
  Pipe pipe;
  Client client(&pipe);
  accept(pipe, client);
  pipe.feed(SERVER_SUSPEND);
  const uint8_t one[] = {0x42};
  pipe.feed(one);
  client.loop();
  client.write_array(one, sizeof(one));
  EXPECT_EQ(client.available(), 1u);
  pipe.up_ = false;
  client.loop();
  // Still readable while down.
  EXPECT_EQ(client.available(), 1u);
  EXPECT_EQ(client.available_for_write(), 0u);
  client.write_array(one, sizeof(one));
  EXPECT_EQ(client.flush(), uart::UARTFlushResult::UART_FLUSH_RESULT_FAILED);
  pipe.clear();
  pipe.up_ = true;
  client.loop();
  EXPECT_EQ(client.available(), 0u);
  const uint8_t offers[] = {0xFF, 0xFB, 0x00, 0xFF, 0xFD, 0x00, 0xFF, 0xFB, 0x2C};
  ASSERT_EQ(pipe.n_, sizeof(offers));
  EXPECT_EQ(std::memcmp(pipe.buf_, offers, sizeof(offers)), 0);
}

TEST(Rfc2217Client, LoopFlushesWhatItWrote) {
  Pipe pipe;
  Client client(&pipe);
  client.loop();
  EXPECT_EQ(pipe.flushes_, 1);
  client.loop();
  EXPECT_EQ(pipe.flushes_, 1);
}

}  // namespace
}  // namespace esphome::rfc2217_uart::testing
