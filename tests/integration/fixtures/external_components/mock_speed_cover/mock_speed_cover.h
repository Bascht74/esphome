#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/core/component.h"

namespace esphome::mock_speed_cover {

// A cover with the speeds "slow" and "fast" that logs each move with its speed and jumps to the target.
class MockSpeedCover : public cover::Cover, public Component {
 public:
  cover::CoverTraits get_traits() override {
    cover::CoverTraits traits;
    traits.set_supports_position(true);
    traits.set_supports_stop(true);
    return traits;
  }

 protected:
  void control(const cover::CoverCall &call) override;
};

}  // namespace esphome::mock_speed_cover
