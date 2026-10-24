#include "mock_speed_cover.h"

#include "esphome/core/log.h"

namespace esphome::mock_speed_cover {

static const char *const TAG = "mock_speed_cover";

void MockSpeedCover::control(const cover::CoverCall &call) {
  const auto &position = call.get_position();
  if (!position.has_value())
    return;
  ESP_LOGI(TAG, "Move to %.2f at speed '%s'", *position, call.get_speed().c_str());
  this->position = *position;
  this->publish_state();
}

}  // namespace esphome::mock_speed_cover
