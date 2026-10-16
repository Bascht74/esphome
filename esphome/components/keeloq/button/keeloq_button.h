#pragma once

#include "esphome/components/button/button.h"
#include "esphome/components/keeloq/keeloq.h"
#include "esphome/core/helpers.h"

namespace esphome::keeloq {

class LearnButton : public button::Button, public Parented<KeeloqRemote>, public remote_base::RemoteTransmittable {
 public:
  void set_channels(uint16_t channels) {
    this->channels_ = channels;
    this->has_channels_ = true;
  }
  void set_legacy(bool legacy) { this->legacy_ = legacy; }

 protected:
  void press_action() override {
    uint16_t channels;
    if (this->parent_->resolve_channels(this->has_channels_, this->channels_, channels)) {
      this->parent_->learn(this->transmitter_, channels, this->legacy_);
    }
  }

  uint16_t channels_{0};
  bool has_channels_{false};
  bool legacy_{false};
};

class TakeOverButton : public button::Button, public Parented<KeeloqRemote> {
 protected:
  void press_action() override { this->parent_->take_over_next_frame(); }
};

}  // namespace esphome::keeloq
