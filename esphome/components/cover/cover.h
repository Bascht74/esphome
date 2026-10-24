#pragma once

#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "esphome/core/string_ref.h"

#include "cover_traits.h"

namespace esphome::cover {

static constexpr float COVER_OPEN = 1.0f;
static constexpr float COVER_CLOSED = 0.0f;

#ifdef USE_COVER_SPEED
/// Speed names of a cover: a codegen table in flash (see ConstVector).
using CoverSpeeds = ConstVector<const char *, true>;
#endif

#define LOG_COVER(prefix, type, obj) \
  if (log_entity_header(TAG, prefix, LOG_STR_LITERAL(type), (obj))) { \
    auto traits_ = (obj)->get_traits(); \
    if (traits_.get_is_assumed_state()) { \
      ESP_LOGCONFIG(TAG, "%s  Assumed State: YES", prefix); \
    } \
    LOG_ENTITY_DEVICE_CLASS(TAG, prefix, *(obj)); \
  }

class Cover;

class CoverCall {
 public:
  CoverCall(Cover *parent);

  /// Set the command as a string, "STOP", "OPEN", "CLOSE", "TOGGLE".
  CoverCall &set_command(const char *command);
  /// Set the command to open the cover.
  CoverCall &set_command_open();
  /// Set the command to close the cover.
  CoverCall &set_command_close();
  /// Set the command to stop the cover.
  CoverCall &set_command_stop();
  /// Set the command to stop the cover tilt.
  CoverCall &set_command_stop_tilt();
  /// Set the command to toggle the cover.
  CoverCall &set_command_toggle();
  /// Set the call to a certain target position.
  CoverCall &set_position(float position);
  /// Set the call to a certain target tilt.
  CoverCall &set_tilt(float tilt);
  /// Set whether this cover call should stop the cover.
  CoverCall &set_stop(bool stop);
  /// Set whether this cover call should stop the cover tilt.
  CoverCall &set_stop_tilt(bool stop_tilt);
#ifdef USE_COVER_SPEED
  /// Set the speed by name; an empty name means the cover's default speed, an unsupported one is dropped.
  CoverCall &set_speed(const char *speed, size_t len);
  CoverCall &set_speed(const char *speed);
  CoverCall &set_speed(const std::string &speed) { return this->set_speed(speed.data(), speed.size()); }
#endif

  /// Perform the cover call.
  void perform();

  const optional<float> &get_position() const;
  bool get_stop() const { return this->stop_; }
  bool get_stop_tilt() const { return this->stop_tilt_; }
  const optional<float> &get_tilt() const;
  const optional<bool> &get_toggle() const;
#ifdef USE_COVER_SPEED
  /// The speed of this move, an entry of the cover's speed list; empty means the default speed.
  StringRef get_speed() const { return StringRef::from_maybe_nullptr(this->speed_); }
#endif

 protected:
  void validate_();

  Cover *parent_;
  bool stop_{false};
  bool stop_tilt_{false};
  optional<float> position_{};
  optional<float> tilt_{};
  optional<bool> toggle_{};
#ifdef USE_COVER_SPEED
  const char *speed_{nullptr};
#endif
};

/// Struct used to store the restored state of a cover
struct CoverRestoreState {
  float position;
  float tilt;

  /// Convert this struct to a cover call that can be performed.
  CoverCall to_call(Cover *cover);
  /// Apply these settings to the cover
  void apply(Cover *cover);
} __attribute__((packed));

/// Enum encoding the current operation of a cover.
enum CoverOperation : uint8_t {
  /// The cover is currently idle (not moving)
  COVER_OPERATION_IDLE = 0,
  /// The cover is currently opening.
  COVER_OPERATION_OPENING,
  /// The cover is currently closing.
  COVER_OPERATION_CLOSING,
};

const LogString *cover_operation_to_str(CoverOperation op);

/** Base class for all cover devices.
 *
 * Covers currently have three properties:
 *  - position - The current position of the cover from 0.0 (fully closed) to 1.0 (fully open).
 *    For covers with only binary OPEN/CLOSED position this will always be either 0.0 or 1.0
 *  - tilt - The tilt value of the cover from 0.0 (closed) to 1.0 (closed)
 *  - current_operation - The operation the cover is currently performing, this can
 *    be one of IDLE, OPENING and CLOSING.
 *
 * For users: All cover operations must be performed over the .make_call() interface.
 * To command a cover, use .make_call() to create a call object, set all properties
 * you wish to set, and activate the command with .perform().
 * For reading out the current values of the cover, use the public .position, .tilt etc
 * properties (these are read-only for users)
 *
 * For integrations: Integrations must implement two methods: control() and get_traits().
 * Control will be called with the arguments supplied by the user and should be used
 * to control all values of the cover. Also implement get_traits() to return what operations
 * the cover supports.
 */
class Cover : public EntityBase {
 public:
  explicit Cover();

  /// The current operation of the cover (idle, opening, closing).
  CoverOperation current_operation{COVER_OPERATION_IDLE};
  /** The position of the cover from 0.0 (fully closed) to 1.0 (fully open).
   *
   * For binary covers this is always equals to 0.0 or 1.0 (see also COVER_OPEN and
   * COVER_CLOSED constants).
   */
  float position;
  /// The current tilt value of the cover from 0.0 to 1.0.
  float tilt{COVER_OPEN};

  /// Construct a new cover call used to control the cover.
  CoverCall make_call() { return {this}; }

  template<typename F> void add_on_state_callback(F &&f) { this->state_callback_.add(std::forward<F>(f)); }

  /** Publish the current state of the cover.
   *
   * First set the .position, .tilt, etc values and then call this method
   * to publish the state of the cover.
   *
   * @param save Whether to save the updated values in RTC area.
   */
  void publish_state(bool save = true);

  virtual CoverTraits get_traits() = 0;

#ifdef USE_COVER_SPEED
  /// Codegen only: points at a table that outlives the cover (see ConstVector::assign_static).
  void set_supported_speeds_static(const char *const *speeds, size_t count) {
    this->supported_speeds_.assign_static(speeds, count);
  }
  /// The speeds this cover supports; empty when it has none.
  const CoverSpeeds &get_supported_speeds() const { return this->supported_speeds_; }
#endif

  /// Helper method to check if the cover is fully open. Equivalent to comparing .position against 1.0
  bool is_fully_open() const { return this->position == COVER_OPEN; }
  /// Helper method to check if the cover is fully closed. Equivalent to comparing .position against 0.0
  bool is_fully_closed() const { return this->position == COVER_CLOSED; }

 protected:
  friend CoverCall;

  virtual void control(const CoverCall &call) = 0;

  optional<CoverRestoreState> restore_state_();

  LazyCallbackManager<void()> state_callback_{};

  ESPPreferenceObject rtc_;
#ifdef USE_COVER_SPEED
  CoverSpeeds supported_speeds_;
#endif
};

}  // namespace esphome::cover
