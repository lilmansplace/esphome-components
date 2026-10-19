#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <string>

namespace esphome {
namespace si4703 {

class Si4703Component : public Component, public i2c::I2CDevice {
 public:
  // Runs before the I2C bus is initialised so the 2-wire-mode reset can drive SDIO directly.
  float get_setup_priority() const override;
  void setup() override;
  void loop() override;
  void dump_config() override;

  // --- configuration (from YAML) ---
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_sdio_pin(InternalGPIOPin *pin) { this->sdio_pin_ = pin; }
  void set_initial_frequency(float mhz) { this->initial_frequency_ = mhz; }
  void set_initial_volume(uint8_t v) { this->current_volume_ = v > 15 ? 15 : v; }
  void set_de_emphasis_50us(bool v) { this->de_50us_ = v; }
  // Seek RSSI threshold (dBuV, 0-127). Can be changed at runtime; applies to the next seek.
  void set_seek_threshold(uint8_t v);
  uint8_t get_seek_threshold() const { return this->seek_threshold_; }
  void set_frequency_sensor(sensor::Sensor *s) { this->frequency_sensor_ = s; }
  void set_rssi_sensor(sensor::Sensor *s) { this->rssi_sensor_ = s; }
  void set_station_name_sensor(text_sensor::TextSensor *s) { this->station_name_sensor_ = s; }
  void set_radio_text_sensor(text_sensor::TextSensor *s) { this->radio_text_sensor_ = s; }

  // --- control API (lambdas / media player) ---
  bool is_ready() const { return this->ready_; }
  void set_frequency(float mhz);
  float get_frequency() const { return this->current_frequency_; }
  void set_volume(uint8_t volume);
  uint8_t get_volume() const { return this->current_volume_; }
  void set_mono(bool mono);
  bool is_mono() const { return this->current_mono_; }
  void mute();
  void unmute();
  void set_mute(bool m) { m ? this->mute() : this->unmute(); }
  bool is_muted() const { return this->muted_; }
  void seek_up() { this->start_seek_(true); }
  void seek_down() { this->start_seek_(false); }
  bool is_seeking() const { return this->seeking_; }
  int get_signal_strength() const { return this->rssi_; }
  bool is_stereo() const { return this->stereo_; }
  // Read and log all 16 registers (for debugging)
  void log_registers();

 protected:
  void hardware_reset_();
  void init_stage1_();  // verify chip, start crystal oscillator
  void init_stage2_();  // power up and configure
  void report_after_settle_(const char *what);
  void start_seek_(bool up);
  void finish_seek_();

  bool read_all_();     // 32 bytes: regs 0x0A..0x0F then 0x00..0x09
  bool read_status_();  // 12 bytes: regs 0x0A..0x0F only
  bool write_regs_();   // regs 0x02..0x07
  bool wait_stc_(bool want_set, uint32_t timeout_ms);
  void refresh_frequency_();
  void process_rds_();
  void clear_rds_();

  GPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *sdio_pin_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};
  text_sensor::TextSensor *station_name_sensor_{nullptr};
  text_sensor::TextSensor *radio_text_sensor_{nullptr};

  uint16_t registers_[16]{0};

  float initial_frequency_{87.5f};
  float current_frequency_{0.0f};
  uint8_t current_volume_{8};
  uint8_t seek_threshold_{10};
  bool de_50us_{false};
  bool current_mono_{false};
  bool muted_{false};
  bool stereo_{false};
  int rssi_{0};

  bool ready_{false};
  uint8_t init_attempts_{0};
  bool seeking_{false};
  uint32_t seek_started_{0};
  uint32_t last_poll_{0};
  uint32_t last_rssi_publish_{0};
  uint32_t last_rssi_forced_{0};
  uint32_t rssi_sum_{0};
  uint32_t rssi_count_{0};
  int last_rssi_published_{-1};
  uint32_t rds_groups_{0};

  // RDS
  char ps_[9]{0};
  uint8_t ps_seen_{0};
  std::string last_ps_;
  char rt_[65]{0};
  uint16_t rt_seen_{0};
  int8_t rt_ab_{-1};
  std::string last_rt_;
  std::string ps_candidate_;
  std::string rt_candidate_;
};

}  // namespace si4703
}  // namespace esphome
