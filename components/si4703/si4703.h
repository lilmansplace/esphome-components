#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/gpio.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/preferences/preferences.h"

namespace esphome {
namespace si4703 {

class Si4703Component : public Component, public i2c::I2CDevice {
 public:
  float get_setup_priority() const override;
  void setup() override;
  void dump_config() override;
  
  // Add reset pin
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  
  // Set/get frequency
  void set_frequency(float frequency);
  float get_frequency() const { return this->current_frequency_; }
  
  // Set/get volume
  void set_volume(uint8_t volume);
  uint8_t get_volume() const { return this->current_volume_; }
  
  // Get mute status
  bool is_muted() const { return this->muted_; }
  
  // Set mono/stereo
  void set_mono(bool mono);
  bool is_mono() const { return this->current_mono_; }
  
  // Mute/unmute methods
  void mute();
  void unmute();
  
  // Tuning methods
  void seek_up();
  void seek_down();
  
  // Add frequency sensor
  void set_frequency_sensor(sensor::Sensor *frequency_sensor) { 
    this->frequency_sensor_ = frequency_sensor; 
  }

  // Update function (called periodically)
  void update();

 protected:
  // Initialize preferences for persistent storage
  void init_preferences_();
  // Load settings from persistent storage
  void load_preferences_();
  // Save settings to persistent storage
  void save_preferences_();

  // Reset the device
  void reset_device_();
  
  // Read/write registers to chip
  bool read_registers_();
  bool write_registers_();
  
  // Update internal state from registers
  void update_internal_state_();
  
  // GPIO pin for reset
  GPIOPin *reset_pin_{nullptr};
  
  // Sensor for reporting frequency
  sensor::Sensor *frequency_sensor_{nullptr};
  
  // Current state variables
  float current_frequency_{87.5f}; // Default to minimum FM band frequency
  uint8_t current_volume_{1};      // Default to low volume
  bool current_mono_{false};       // Default to stereo
  bool muted_{false};              // Default to unmuted
  
  // Storage for chip registers (16 registers, each 16-bit)
  uint16_t registers_[16]{0};
  
  // Preferences for saving settings
  preferences::Preferences pref_;
  bool preferences_initialized_{false};
};

} // namespace si4703
} // namespace esphome 