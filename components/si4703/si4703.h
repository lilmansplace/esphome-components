#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace si4703 {

class Si4703Component : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // --- Radio Control Methods (Add more as needed) ---
  void set_frequency(float frequency);
  void set_volume(uint8_t volume); // Volume usually 0-15 for Si4703
  void seek_up();
  void seek_down();
  void set_mono(bool mono);
  void mute();
  void unmute();

 protected:
  // Internal helper methods for I2C communication and chip control
  bool write_registers_();
  bool read_registers_();

  // Internal state variables
  uint16_t registers_[16]; // Si4703 has 16 registers (0x00 to 0x0F)
  float current_frequency_{0.0f};
  uint8_t current_volume_{0};
  bool current_mono_{false};
  bool muted_{false};
};

} // namespace si4703
} // namespace esphome 