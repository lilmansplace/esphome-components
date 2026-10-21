#pragma once

#include <cmath>

#include "esphome/components/number/number.h"
#include "esphome/core/preferences.h"
#include "../si4703.h"

namespace esphome::si4703 {

/// Base for the tuner's numbers: optionally saves the value whenever it changes and restores it at boot.
class Si4703Number : public number::Number, public Parented<Si4703Component> {
 public:
  void set_restore_value(bool restore_value) { this->restore_value_ = restore_value; }
  /// Loads the saved value. Returns false when restore_value is off or nothing has been saved yet.
  bool load_value(float *value) {
    if (!this->restore_value_)
      return false;
    this->pref_ = this->make_entity_preference<float>();
    if (!this->pref_.load(value))
      return false;
    this->saved_ = *value;
    return true;
  }
  /// Publishes a value reported by the tuner and saves it when restore_value is on.
  void publish_and_save(float value) {
    this->publish_state(value);
    if (this->restore_value_ && value != this->saved_) {
      this->saved_ = value;
      this->pref_.save(&value);
    }
  }

 protected:
  ESPPreferenceObject pref_;
  float saved_{NAN};
  bool restore_value_{false};
};

class FrequencyNumber final : public Si4703Number {
 protected:
  void control(float value) override { this->parent_->set_frequency(value); }
};

class VolumeNumber final : public Si4703Number {
 protected:
  void control(float value) override { this->parent_->set_volume(static_cast<uint8_t>(value)); }
};

class SeekThresholdNumber final : public Si4703Number {
 protected:
  void control(float value) override { this->parent_->set_seek_threshold(static_cast<uint8_t>(value)); }
};

}  // namespace esphome::si4703
