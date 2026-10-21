#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#include <functional>

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::si4703 {

// Si4702/Si4703 FM tuner. Datasheet: Si4702/03-C19; programming guide: Silicon Labs AN230.

enum Band : uint8_t {
  BAND_87_5_108 = 0,  // USA, Europe
  BAND_76_108 = 1,    // Japan wide band
  BAND_76_90 = 2,     // Japan
};

enum ChannelSpacing : uint8_t {
  SPACING_200KHZ = 0,  // USA, Australia
  SPACING_100KHZ = 1,  // Europe, Japan
  SPACING_50KHZ = 2,
};

static constexpr uint8_t MAX_SCAN_RESULTS = 32;

struct ScanResult {
  float frequency;
  uint8_t rssi;
};

enum class State : uint8_t {
  INIT,      // waiting for setup timeouts (reset, oscillator, power-up)
  IDLE,      // powered and tuned, polling status/RDS
  TUNING,    // TUNE bit set, waiting for STC
  SEEKING,   // SEEK bit set, waiting for STC
  CLEARING,  // TUNE/SEEK bit cleared, waiting for STC to drop
};

class Si4703Component : public Component, public i2c::I2CDevice {
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(stereo)
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(frequency)
  SUB_SENSOR(rssi)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(station_name)
  SUB_TEXT_SENSOR(radio_text)
#endif
#ifdef USE_NUMBER
  SUB_NUMBER(frequency)
  SUB_NUMBER(volume)
  SUB_NUMBER(seek_threshold)
#endif
#ifdef USE_SWITCH
  SUB_SWITCH(mute)
  SUB_SWITCH(mono)
#endif
#ifdef USE_BUTTON
  SUB_BUTTON(seek_up)
  SUB_BUTTON(seek_down)
  SUB_BUTTON(scan)
#endif

 public:
  // Must run before the I2C bus so the bus-mode reset can drive SDIO as a plain GPIO.
  float get_setup_priority() const override { return setup_priority::BUS + 1.0f; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_sdio_pin(InternalGPIOPin *pin) { this->sdio_pin_ = pin; }
  void set_band(Band band) { this->band_ = band; }
  void set_channel_spacing(ChannelSpacing spacing) { this->spacing_ = spacing; }
  void set_de_emphasis_50us(bool de_50us) { this->de_50us_ = de_50us; }
  void set_initial_frequency(float mhz) { this->target_frequency_ = mhz; }
  void set_initial_volume(uint8_t volume) { this->volume_ = volume > 15 ? 15 : volume; }

  // Control. All calls are non-blocking; tune/seek complete over the next loop iterations.
  void set_frequency(float mhz);
  void seek_up() { this->start_seek_(true); }
  void seek_down() { this->start_seek_(false); }
  void set_volume(uint8_t volume);
  void set_mute(bool mute);
  void set_mono(bool mono);
  void set_seek_threshold(uint8_t threshold);

  // Band scan: seeks through the whole band and records every station found (frequency + RSSI).
  // Results are sorted strongest first; the previous frequency is restored when the scan ends.
  void start_scan();
  bool is_scanning() const { return this->scanning_; }
  /// True while initialising, tuning, seeking or scanning (new commands would be ignored)
  bool is_busy() const { return this->state_ != State::IDLE || this->scanning_; }
  uint8_t get_scan_count() const { return this->scan_count_; }
  float get_scan_frequency(uint8_t index) const {
    return index < this->scan_count_ ? this->scan_results_[index].frequency : 0.0f;
  }
  uint8_t get_scan_rssi(uint8_t index) const { return index < this->scan_count_ ? this->scan_results_[index].rssi : 0; }
  void add_on_scan_complete_callback(std::function<void()> &&callback) {
    this->scan_complete_callback_.add(std::move(callback));
  }

  // State
  float get_frequency() const { return this->frequency_; }
  uint8_t get_volume() const { return this->volume_; }
  uint8_t get_seek_threshold() const { return this->seek_threshold_; }
  bool is_muted() const { return this->muted_; }
  bool is_mono() const { return this->mono_; }
  bool is_stereo() const { return this->stereo_; }
  uint8_t get_rssi() const { return this->rssi_; }
  float get_band_min() const;
  float get_band_max() const;
  float get_channel_step() const;

 protected:
  void hardware_reset_();
  void init_verify_();
  void init_power_up_();
  void init_finish_();

  bool read_all_();
  bool read_status_();
  bool write_registers_();
  void update_from_status_();
  void complete_tune_();
  void start_seek_(bool up);
  void tune_(float mhz);
  void issue_seek_(bool up, bool stop_at_band_limit);
  bool last_op_seek_{false};
  void publish_tuned_();
  void clear_rds_();
  void process_rds_();
  void scan_step_();
  void finish_scan_();

  bool scanning_{false};
  float scan_return_frequency_{0.0f};
  ScanResult scan_results_[MAX_SCAN_RESULTS]{};
  uint8_t scan_count_{0};
  uint8_t stc_rssi_{0};
  bool stc_band_limit_{false};
  CallbackManager<void()> scan_complete_callback_;

  GPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *sdio_pin_{nullptr};

  uint16_t registers_[16]{};

  State state_{State::INIT};
  uint16_t chip_id_{0};
  uint16_t test1_reset_{0};
  uint32_t last_poll_{0};
  uint32_t op_started_{0};
  uint32_t last_rssi_publish_{0};
  uint32_t settle_until_{0};
  uint8_t init_attempts_{0};

  Band band_{BAND_87_5_108};
  ChannelSpacing spacing_{SPACING_100KHZ};
  bool de_50us_{false};
  float target_frequency_{87.5f};
  float frequency_{0.0f};
  uint8_t volume_{8};
  uint8_t seek_threshold_{10};
  bool muted_{false};
  bool mono_{false};
  bool stereo_{false};
  uint8_t rssi_{0};
  int16_t last_published_rssi_{-1};

  // RDS (fixed buffers, no heap use after setup)
  char ps_[9]{};
  char ps_candidate_[9]{};
  char ps_published_[9]{};
  uint8_t ps_seen_{0};
  char rt_[65]{};
  char rt_candidate_[65]{};
  char rt_published_[65]{};
  uint16_t rt_seen_{0};
  int8_t rt_ab_{-1};
};

}  // namespace esphome::si4703
