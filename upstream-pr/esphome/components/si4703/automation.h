#pragma once

#include "esphome/core/automation.h"
#include "si4703.h"

namespace esphome::si4703 {

template<typename... Ts> class TuneAction final : public Action<Ts...>, public Parented<Si4703Component> {
 public:
  TEMPLATABLE_VALUE(float, frequency)

  void play(const Ts &...x) override { this->parent_->set_frequency(this->frequency_.value(x...)); }
};

template<typename... Ts> class SeekAction final : public Action<Ts...>, public Parented<Si4703Component> {
 public:
  explicit SeekAction(bool up) : up_(up) {}

  void play(const Ts &...x) override {
    if (this->up_) {
      this->parent_->seek_up();
    } else {
      this->parent_->seek_down();
    }
  }

 protected:
  bool up_;
};

template<typename... Ts> class SetVolumeAction final : public Action<Ts...>, public Parented<Si4703Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, volume)

  void play(const Ts &...x) override { this->parent_->set_volume(this->volume_.value(x...)); }
};

class ScanCompleteTrigger final : public Trigger<> {
 public:
  explicit ScanCompleteTrigger(Si4703Component *parent) {
    parent->add_on_scan_complete_callback([this]() { this->trigger(); });
  }
};

template<typename... Ts> class ScanAction final : public Action<Ts...>, public Parented<Si4703Component> {
 public:
  void play(const Ts &...x) override { this->parent_->start_scan(); }
};

}  // namespace esphome::si4703
