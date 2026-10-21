#pragma once

#include "esphome/components/number/number.h"
#include "../si4703.h"

namespace esphome::si4703 {

class FrequencyNumber final : public number::Number, public Parented<Si4703Component> {
 protected:
  void control(float value) override { this->parent_->set_frequency(value); }
};

class VolumeNumber final : public number::Number, public Parented<Si4703Component> {
 protected:
  void control(float value) override { this->parent_->set_volume(static_cast<uint8_t>(value)); }
};

class SeekThresholdNumber final : public number::Number, public Parented<Si4703Component> {
 protected:
  void control(float value) override { this->parent_->set_seek_threshold(static_cast<uint8_t>(value)); }
};

}  // namespace esphome::si4703
