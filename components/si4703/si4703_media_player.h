#pragma once

#include "esphome/components/media_player/media_player.h"
#include "si4703.h"

namespace esphome {
namespace si4703 {

class Si4703MediaPlayer : public media_player::MediaPlayer, public Component {
 public:
  Si4703MediaPlayer(const std::string &name) : name_(name) {}

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  media_player::MediaPlayerTraits get_traits() override;

  void set_parent(Si4703Component *parent) { this->parent_ = parent; }

 protected:
  void control(const media_player::MediaPlayerCall &call) override;

  std::string name_;
  Si4703Component *parent_{nullptr};
};

} // namespace si4703
} // namespace esphome 