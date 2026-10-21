#pragma once

#include "esphome/components/button/button.h"
#include "../si4703.h"

namespace esphome::si4703 {

class SeekUpButton final : public button::Button, public Parented<Si4703Component> {
 protected:
  void press_action() override { this->parent_->seek_up(); }
};

class SeekDownButton final : public button::Button, public Parented<Si4703Component> {
 protected:
  void press_action() override { this->parent_->seek_down(); }
};

class ScanButton final : public button::Button, public Parented<Si4703Component> {
 protected:
  void press_action() override { this->parent_->start_scan(); }
};

}  // namespace esphome::si4703
