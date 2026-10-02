#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/socket/tcp_client_link.h"
#ifdef USE_SOCKET_TCP_LISTENER
#include "esphome/components/socket/tcp_listener.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#ifdef USE_UART_TCP_NOISE
#include "esphome/components/noise/uart_channel.h"
#endif

#include <cstdint>
#include <memory>

namespace esphome::noise {
template<typename Self> bool uart_noise_ready(Self *self, const char *tag);
}  // namespace esphome::noise

namespace esphome::uart_tcp {

/// Copies raw bytes between one hardware UART and one TCP socket.
class UartTcp : public Component, public uart::UARTDevice {
 public:
  void set_host(const char *host) { this->link_.set_host(host); }
  void set_port(uint16_t port) { this->link_.set_port(port); }
  void set_reconnect_interval(uint32_t ms) { this->link_.set_reconnect_interval(ms); }
  void set_timeout(uint32_t ms) { this->timeout_ms_ = ms; }
  void set_connected_sensor(binary_sensor::BinarySensor *sensor) { this->connected_sensor_ = sensor; }
#ifdef USE_SOCKET_TCP_LISTENER
  void set_server(bool server) { this->server_ = server; }
#ifdef USE_SOCKET_IPV4_ALLOW
  void set_allow(const socket::Ipv4AllowEntry *entries, size_t count) { this->listener_.set_allow(entries, count); }
#endif
#endif

  void setup() override;
  void loop() override;
  void dump_config() override;
  template<typename Self> friend bool noise::uart_noise_ready(Self *self, const char *tag);
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

 protected:
  void sync_link_();
  bool maintain_link_();
  void read_socket_();
  void read_uart_();
  void discard_uart_();
  void check_timeout_();
  void note_io_();
#ifdef USE_UART_TCP_NOISE
  virtual ssize_t read_link_(uint8_t *buf, size_t len) { return this->link_.read(buf, len); }
  virtual size_t write_link_(const uint8_t *data, size_t len) { return this->link_.queue(data, len); }
  virtual size_t wire_budget_() const { return this->link_.tx_free(); }
#else
  ssize_t read_link_(uint8_t *buf, size_t len) { return this->link_.read(buf, len); }
  size_t write_link_(const uint8_t *data, size_t len) { return this->link_.queue(data, len); }
  size_t wire_budget_() const { return this->link_.tx_free(); }
#endif

  static constexpr size_t READ_CHUNK = 128;

  socket::TcpClientLink link_;
#ifdef USE_SOCKET_TCP_LISTENER
  socket::TcpListener listener_;
#endif
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  uint32_t timeout_ms_{0};
  uint32_t last_io_ms_{0};
  bool server_{false};
  // The link state loop() saw last; edges clear the buffer and publish the sensor.
  bool link_was_up_{false};
  // A read stopped before EAGAIN. ready() stays false until new data arrives.
  bool rx_pending_{false};
};

#ifdef USE_UART_TCP_MODBUS
/// Same bridge, but the socket speaks Modbus TCP and the pins stay RTU.
class UartTcpModbus : public UartTcp {
 public:
  void set_send_wait_time(uint32_t ms) { this->send_wait_ms_ = ms; }

  void loop() override;
  void dump_config() override;

  template<typename Self> friend bool noise::uart_noise_ready(Self *self, const char *tag);

 protected:
  void pump_modbus_();
  void read_tcp_buf_();
  void pull_uart_buf_();
  bool take_rtu_(uint8_t *pdu, size_t *pdu_len, uint8_t *unit);
  // True once the whole frame is in the driver. What does not fit stays queued.
  bool write_rtu_(const uint8_t *pdu, size_t pdu_len, uint8_t unit);
  bool drain_rtu_();
  void send_mbap_(uint16_t txn, uint8_t unit, const uint8_t *pdu, size_t pdu_len);
  uint32_t frame_gap_us_() const;

  static constexpr size_t TCP_FRAME_SIZE = 260;
  static constexpr size_t RTU_FRAME_SIZE = 256;

  uint32_t last_uart_us_{0};
  uint32_t wait_started_ms_{0};
  uint32_t send_wait_ms_{2000};
  uint16_t txn_{0};
  uint16_t tcp_len_{0};
  uint16_t uart_len_{0};
  uint16_t rtu_tx_len_{0};
  uint16_t rtu_tx_off_{0};
  uint8_t pending_unit_{0};
  uint8_t pending_function_{0};
  bool wait_uart_{false};
  bool wait_tcp_{false};
  // The request is still leaving the driver; arm the UART wait once it has.
  bool arm_wait_uart_{false};
  uint8_t tcp_buf_[TCP_FRAME_SIZE]{};
  uint8_t uart_buf_[RTU_FRAME_SIZE]{};
  uint8_t rtu_tx_[RTU_FRAME_SIZE]{};
};
#endif

#ifdef USE_UART_TCP_NOISE
class UartTcpNoise : public UartTcp {
 public:
  void set_noise_psk(const uint8_t *psk) { this->channel_.set_psk(psk); }
  void set_noise_initiator(bool initiator) { this->channel_.set_initiator(initiator); }
  void loop() override;
  void dump_config() override;

  template<typename Self> friend bool noise::uart_noise_ready(Self *self, const char *tag);

 protected:
  ssize_t read_link_(uint8_t *buf, size_t len) override { return this->channel_.read(this->link_, buf, len); }
  size_t write_link_(const uint8_t *data, size_t len) override { return this->channel_.write(this->link_, data, len); }
  size_t wire_budget_() const override { return this->channel_.plain_budget(this->link_.tx_free()); }

  noise::UartChannel channel_;
};

#ifdef USE_UART_TCP_MODBUS
class UartTcpModbusNoise : public UartTcpModbus {
 public:
  UartTcpModbusNoise() { this->channel_.set_modbus(true); }
  void set_noise_psk(const uint8_t *psk) { this->channel_.set_psk(psk); }
  void set_noise_initiator(bool initiator) { this->channel_.set_initiator(initiator); }
  void loop() override;
  void dump_config() override;

  template<typename Self> friend bool noise::uart_noise_ready(Self *self, const char *tag);

 protected:
  ssize_t read_link_(uint8_t *buf, size_t len) override { return this->channel_.read(this->link_, buf, len); }
  size_t write_link_(const uint8_t *data, size_t len) override { return this->channel_.write(this->link_, data, len); }
  size_t wire_budget_() const override { return this->channel_.plain_budget(this->link_.tx_free()); }

  noise::UartChannel channel_;
};
#endif
#endif

}  // namespace esphome::uart_tcp
