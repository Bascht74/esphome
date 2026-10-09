#pragma once

#include "telnet.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/uart/uart_virtual.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include <cstddef>
#include <cstdint>

namespace esphome::tcp_uart {
class TcpUart;
}  // namespace esphome::tcp_uart

namespace esphome::rfc2217_uart {

/// [RFC 2217] SET-PARITY value of a UART parity.
inline uint8_t to_rfc_parity(uart::UARTParityOptions parity) {
  switch (parity) {
    case uart::UART_CONFIG_PARITY_ODD:
      return PARITY_ODD;
    case uart::UART_CONFIG_PARITY_EVEN:
      return PARITY_EVEN;
    default:
      return PARITY_NONE;
  }
}

/// Telnet with the COM-PORT option on a tcp_uart: option negotiation, escaping and RFC 2217 flow control.
class Rfc2217Base : public Component {
 public:
  void set_tcp_uart(tcp_uart::TcpUart *tcp) { this->tcp_ = tcp; }
  // Same priority as tcp_uart, which is registered first: it loops first and sees each link edge first.
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

 protected:
  /// [RFC 1143] option state, without the queue.
  enum class OptionState : uint8_t {
    NO,
    YES,
    WANT_YES,
  };

  /// Runs the link edges; true while the link is up.
  bool sync_link_();
  /// Takes received bytes while the payload has room. Payload goes to deliver(), COM-PORT commands to on_command().
  void read_tcp_();
  void write_tcp_(const uint8_t *data, size_t len);
  void flush_tcp_();
  /// Sends a COM-PORT command; the server's codes get SERVER_OFFSET. False without room.
  bool send_command_(uint8_t code, const uint8_t *value, size_t len);
  /// RFC 2217 flow control on the payload buffer that the peer fills.
  void update_flow_(size_t level, size_t capacity);
  /// COM-PORT is enabled in the direction RFC 2217 uses: client WILL, server DO.
  bool com_port_() const;
  void note_drop_(const LogString *message);

  virtual size_t payload_room() = 0;
  virtual void deliver(const uint8_t *data, size_t len) = 0;
  virtual void on_command(uint8_t code, const uint8_t *value, size_t len) = 0;
  virtual void on_link(bool up) = 0;

  void on_option_(uint8_t verb, uint8_t option);
  void send_option_(uint8_t verb, uint8_t option);

  static constexpr size_t READ_CHUNK = 128;

  tcp_uart::TcpUart *tcp_{nullptr};
  TelnetDecoder decoder_;
  // One rate limit for all drop warnings.
  uint32_t drop_log_ms_{0};
  // Index 0 BINARY, 1 COM-PORT; us_ is this side's option, him_ the peer's.
  OptionState us_[2]{};
  OptionState him_[2]{};
  bool server_{false};
  bool link_was_up_{false};
  bool wrote_{false};
  // The peer sent FLOWCONTROL-SUSPEND: hold the payload for it.
  bool peer_suspended_{false};
  // This side sent FLOWCONTROL-SUSPEND.
  bool suspended_peer_{false};
};

/// RFC 2217 client: a UART whose line settings go to the access server.
class Rfc2217Client : public uart::VirtualUARTComponent, public Rfc2217Base {
 public:
  Rfc2217Client() : VirtualUARTComponent(RX_SIZE) {}

  void loop() override;
  void dump_config() override;

  void write_array(const uint8_t *data, size_t len) override;
  size_t available_for_write() override;
  uart::UARTFlushResult flush() override;
  bool is_connected() override;
#if defined(USE_ESP8266) || defined(USE_ESP32)
  using UARTComponent::load_settings;
  // Sends the line settings to the server.
  void load_settings(bool dump_config) override { this->send_settings_(); }
#endif

 protected:
  void send_settings_();
  void send_tx_();
  void check_answer_(const LogString *command, uint32_t asked, uint32_t got);
  size_t payload_room() override { return RX_SIZE - this->available(); }
  void deliver(const uint8_t *data, size_t len) override;
  void on_command(uint8_t code, const uint8_t *value, size_t len) override;
  void on_link(bool up) override;

  static constexpr uint16_t RX_SIZE = 256;
  static constexpr size_t TX_SIZE = 256;
  // Four commands: the baud rate with every byte doubled, three single bytes.
  static constexpr size_t SETTINGS_SIZE = COM_PORT_COMMAND_MAX + 3 * 6;

  // tx_[0, tx_len_): written, not sent; held while the peer has suspended.
  uint16_t tx_len_{0};
  bool settings_pending_{false};
  uint8_t tx_[TX_SIZE]{};
};

/// RFC 2217 access server for a hardware UART.
class Rfc2217Server : public Rfc2217Base, public uart::UARTDevice {
 public:
  Rfc2217Server() { this->server_ = true; }

  void loop() override;
  void dump_config() override;

 protected:
  /// Loads changed line settings into the UART; false where the platform cannot change them at runtime.
  virtual bool load_serial();
  void apply_settings_(uint32_t baud_rate, uint8_t data_bits, uart::UARTParityOptions parity, uint8_t stop_bits);
  void answer_(uint8_t code, uint8_t value);
  void write_serial_();
  size_t serial_room_();
  void read_serial_();
  void discard_serial_();
  size_t payload_room() override { return TO_SERIAL_SIZE - this->to_serial_len_; }
  void deliver(const uint8_t *data, size_t len) override;
  void on_command(uint8_t code, const uint8_t *value, size_t len) override;
  void on_link(bool up) override;

  static constexpr size_t TO_SERIAL_SIZE = 256;

  // Loop start time of the last write to the UART; sizes the next paced write.
  uint32_t last_write_ms_{0};
  // to_serial_[0, to_serial_len_): payload from the client that waits for the UART.
  uint16_t to_serial_len_{0};
  uint8_t to_serial_[TO_SERIAL_SIZE]{};
};

}  // namespace esphome::rfc2217_uart
