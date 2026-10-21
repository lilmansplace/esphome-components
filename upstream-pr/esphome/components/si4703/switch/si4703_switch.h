#pragma once

#include "esphome/components/switch/switch.h"
#include "../si4703.h"

namespace esphome::si4703 {

class MuteSwitch final : public switch_::Switch, public Parented<Si4703Component> {
 protected:
  void write_state(bool state) override { this->parent_->set_mute(state); }
};

class MonoSwitch final : public switch_::Switch, public Parented<Si4703Component> {
 protected:
  void write_state(bool state) override { this->parent_->set_mono(state); }
};

}  // namespace esphome::si4703
