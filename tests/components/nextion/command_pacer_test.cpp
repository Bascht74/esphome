#include <gtest/gtest.h>

#include <new>
#include <string>
#include <vector>

#include "esphome/components/nextion/nextion.h"
#include "esphome/components/uart/uart_component.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"

namespace esphome::nextion::testing {

// Explicit instantiation skips access checks, which lets the test run the pending sends the way
// Nextion::loop() does, without making them public
template<void (Nextion::*P)(), void (Nextion::*R)(bool), bool NextionBase::*U> struct PacerAccess {
  friend void send_pending(Nextion &nextion) { (nextion.*P)(); }
  friend void clear_queue(Nextion &nextion) { (nextion.*R)(false); }
  friend void mark_ready(Nextion &nextion) { nextion.*U = true; }
};
void send_pending(Nextion &nextion);
void clear_queue(Nextion &nextion);
void mark_ready(Nextion &nextion);
template struct PacerAccess<&Nextion::process_pending_in_queue_, &Nextion::reset_, &Nextion::is_setup_>;

/// Sets the loop clock the way the main loop does at the start of a component's loop()
static void start_loop_pass() { LoopBlockingGuard{nullptr, nullptr, millis()}; }

/// Records each command the component writes; the display never answers
class RecordingUART : public uart::UARTComponent {
 public:
  void write_array(const uint8_t *data, size_t len) override {
    for (size_t i = 0; i < len; i++) {
      if (data[i] != 0xFF) {
        this->current_ += static_cast<char>(data[i]);
      } else if (!this->current_.empty()) {
        // The first of the three 0xFF bytes ends the command
        this->commands.push_back(this->current_);
        this->current_.clear();
      }
    }
  }
  bool peek_byte(uint8_t *data) override { return false; }
  bool read_array(uint8_t *data, size_t len) override { return false; }
  size_t available() override { return 0; }
  uart::UARTFlushResult flush() override { return uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS; }
#if defined(USE_ESP8266) || defined(USE_ESP32)
  void load_settings(bool dump_config) override {}
#endif  // defined(USE_ESP8266) || defined(USE_ESP32)

  std::vector<std::string> commands;

 protected:
  void check_logger_conflict() override {}

  std::string current_;
};

class CommandPacer : public ::testing::Test {
 protected:
  void SetUp() override {
    // A fresh App, so the only timer in its scheduler is the one this test plans
    App.~Application();
    new (&App) Application();
    this->nextion_.set_uart_parent(&this->uart_);
    mark_ready(this->nextion_);
  }

  void TearDown() override {
    clear_queue(this->nextion_);
    App.~Application();
    new (&App) Application();  // drops a timer left by a failed test; later tests use App too
  }

  /// Waits for the planned wake-up and runs only the scheduler, as a main loop pass without component loops does
  static void run_wake_up() {
    App.scheduler.process_to_add();
    const auto wake_in = App.scheduler.next_schedule_in(millis());
    ASSERT_TRUE(wake_in.has_value());
    delay(*wake_in);
    App.scheduler.call(millis());
  }

  RecordingUART uart_;
  Nextion nextion_;
};

// command_spacing sets the minimum time between two commands. A command the pacer holds back must go out
// when that time has passed, not only at the next loop pass, which can be up to loop_interval (16 ms) later.
TEST_F(CommandPacer, HeldBackCommandIsSentWhenSpacingHasPassed) {
  this->nextion_.set_command_spacing(100);

  // 0 ms: the first command goes out, the second has to wait 100 ms
  start_loop_pass();
  this->nextion_.send_command("page 1");
  this->nextion_.send_command("vis b0,1");
  EXPECT_EQ(this->uart_.commands, std::vector<std::string>{"page 1"});

  // 10 ms: a loop pass still holds the command back, and the wake-up stays due at 100 ms
  delay(10);
  start_loop_pass();
  send_pending(this->nextion_);
  EXPECT_EQ(this->uart_.commands, std::vector<std::string>{"page 1"});

  App.scheduler.process_to_add();
  const auto wake_in = App.scheduler.next_schedule_in(millis());
  ASSERT_TRUE(wake_in.has_value());
  EXPECT_GT(*wake_in, 0u);
  EXPECT_LE(*wake_in, 90u);

  // 100 ms: only the scheduler runs, no component loop, and the wake-up sends the command
  delay(*wake_in);
  App.scheduler.call(millis());
  EXPECT_EQ(this->uart_.commands, (std::vector<std::string>{"page 1", "vis b0,1"}));

  // Nothing waits any more, so no further wake-up is planned
  EXPECT_FALSE(App.scheduler.next_schedule_in(millis()).has_value());
}

// A command the pacer holds back right when it is queued, such as one from Home Assistant, plans the wake-up
// itself and goes out without waiting for a loop pass
TEST_F(CommandPacer, CommandHeldBackWhenQueuedIsSentWithoutLoopPass) {
  this->nextion_.set_command_spacing(10);

  start_loop_pass();
  this->nextion_.send_command("page 1");
  this->nextion_.send_command("vis b0,1");
  EXPECT_EQ(this->uart_.commands, std::vector<std::string>{"page 1"});

  run_wake_up();
  EXPECT_EQ(this->uart_.commands, (std::vector<std::string>{"page 1", "vis b0,1"}));
  EXPECT_FALSE(App.scheduler.next_schedule_in(millis()).has_value());
}

// A wake-up sends one command and plans the next one while more wait
TEST_F(CommandPacer, EachWakeUpSendsOneCommandAndPlansTheNext) {
  this->nextion_.set_command_spacing(10);

  start_loop_pass();
  this->nextion_.send_command("page 1");
  this->nextion_.send_command("vis b0,1");
  this->nextion_.send_command("vis b1,1");
  EXPECT_EQ(this->uart_.commands, std::vector<std::string>{"page 1"});

  run_wake_up();
  EXPECT_EQ(this->uart_.commands, (std::vector<std::string>{"page 1", "vis b0,1"}));

  run_wake_up();
  EXPECT_EQ(this->uart_.commands, (std::vector<std::string>{"page 1", "vis b0,1", "vis b1,1"}));
  EXPECT_FALSE(App.scheduler.next_schedule_in(millis()).has_value());
}

}  // namespace esphome::nextion::testing
